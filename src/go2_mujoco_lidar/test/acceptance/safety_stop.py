"""G04 simulation 정지: Nav2 취소와 bridge zero를 먼저 수행한다.

GT의 .25초 차분 선속도 <.03 m/s, 각속도 <.10 rad/s를 연속 1 sim s
확인해야 lifecycle 종료를 시작한다. zero timer는 lifecycle 종료 중에도
유지한다. 독립 native physics 전체 구간으로 clearance와 이동을 판정한다.
실물 정지 기능이나 정책 변경은 포함하지 않는다.
"""
import math
import time

LINEAR_LIMIT = .03
ANGULAR_LIMIT = .10
HOLD_SECONDS = 1.
DISPLACEMENT_LIMIT = .10
CLEARANCE_LOSS_LIMIT = .05
ACTIONS = ("navigate_to_pose", "navigate_through_poses", "follow_path",
           "compute_path_to_pose", "compute_path_through_poses", "spin", "backup", "wait")


def heading(row):
    return math.atan2(2*(row["qw"]*row["qz"]+row["qx"]*row["qy"]),
                      1-2*(row["qy"]**2+row["qz"]**2))


def stationary(rows):
    """마지막 GT와 .25초 이전 표본으로 gait 진동과 실제 이동을 구별한다."""
    if not rows:
        return False
    now = rows[-1]
    old = next((r for r in reversed(rows) if now["sim_time"]-r["sim_time"] >= .25), None)
    if old is None:
        return False
    dt = now["sim_time"]-old["sim_time"]
    speed = math.hypot(now["x"]-old["x"], now["y"]-old["y"])/dt
    angle = heading(now)-heading(old)
    angular = abs(math.atan2(math.sin(angle), math.cos(angle)))/dt
    return speed < LINEAR_LIMIT and angular < ANGULAR_LIMIT


def post_stop_summary(physics, start, stopped, end=math.inf):
    """경보 또는 중지 요청부터 physics 종료까지 매 frame의 최악값을 보존한다."""
    rows = [r for r in physics if start["sim_time"] <= float(r["sim_time"]) <= end]
    if not rows:
        return {"passed": False, "frames": 0}
    alert = rows[0]
    initial = float(alert["min_clearance_m"])
    minimum = min(float(r["min_clearance_m"]) for r in rows)
    displacement = max(math.hypot(float(r["x"])-start["x"], float(r["y"])-start["y"])
                       for r in rows)
    return {"frames": len(rows), "alert_clearance_m": initial,
            "min_clearance_m": minimum, "clearance_loss_m": initial-minimum,
            "max_displacement_m": displacement, "stationary_verified": stopped,
            "passed": bool(stopped) and displacement <= DISPLACEMENT_LIMIT
                      and minimum >= initial-CLEARANCE_LOSS_LIMIT}


class SafetyStop:
    """취소 client는 Nav2 시작 전 준비하고 zero publisher는 정지 때만 만든다."""

    def __init__(self, node):
        from action_msgs.srv import CancelGoal
        self.node = node
        self.clients = {name: node.create_client(CancelGoal, f"/{name}/_action/cancel_goal")
                        for name in ACTIONS}
        self.publisher = None
        self.timer = None
        self.result = None
        self.start = None

    def run(self, truth_rows, pump):
        from action_msgs.srv import CancelGoal
        from geometry_msgs.msg import Twist
        self.start = truth_rows()[-1]
        self.result = {"request_steady_ns": time.monotonic_ns(), "start": self.start,
                       "cancellations": {}, "stationary_verified": False}
        futures = {name: client.call_async(CancelGoal.Request())
                   for name, client in self.clients.items() if client.service_is_ready()}
        self.publisher = self.node.create_publisher(Twist, "/go2_nav2/shadow_cmd_vel", 10)
        self.publisher.publish(Twist())
        self.timer = self.node.create_timer(.02, lambda: self.publisher.publish(Twist()))
        self.result["first_zero_steady_ns"] = time.monotonic_ns()
        deadline = time.monotonic()+15
        since = None
        while time.monotonic() < deadline:
            pump()
            self.publisher.publish(Twist())
            rows = truth_rows()
            now = rows[-1]["sim_time"]
            if stationary(rows):
                since = now if since is None else since
            else:
                since = None
            if since is not None and now-since >= HOLD_SECONDS and all(f.done() for f in futures.values()):
                self.result.update(stationary_verified=True, stationary_since_sim=since,
                                   verified_sim_time=now, verified_steady_ns=time.monotonic_ns())
                break
        for name, future in futures.items():
            self.result["cancellations"][name] = (
                {"return_code": future.result().return_code,
                 "goals_canceling": len(future.result().goals_canceling)}
                if future.done() and future.exception() is None else {"incomplete": True})
        self.result["zero_endpoints"] = [
            {"gid": bytes(e.endpoint_gid).hex(),
             "node": e.node_namespace.rstrip("/")+"/"+e.node_name}
            for e in self.node.get_publishers_info_by_topic("/go2_nav2/shadow_cmd_vel")
            if e.node_name == self.node.get_name()]
        return self.result

    def release(self):
        """정지 검증 뒤 다음 사전 목표를 시작할 때만 zero 소유권을 해제한다."""
        self.node.destroy_timer(self.timer)
        self.node.destroy_publisher(self.publisher)
        self.timer = self.publisher = None

