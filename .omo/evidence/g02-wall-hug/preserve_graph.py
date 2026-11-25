"""Preserve mapper state and fixed relative graph factors, without control I/O.

Serialize once while mapping and again immediately when the independent driver
finishes. The client is already discovered before the finish event, so the final
request precedes the ordinary map-save/mapper teardown.
"""
import json
from pathlib import Path
import time
import rclpy
from slam_toolbox.srv import SerializePoseGraph

ROOT=Path(__file__).resolve().parent
attempt=json.loads((ROOT/"result.json").read_text())["attempt"]
rclpy.init()
node=rclpy.create_node(f"g02_graph_evidence_{attempt}")
client=node.create_client(SerializePoseGraph,"/slam_toolbox/serialize_map")
deadline=time.monotonic()+900
def save(suffix):
    request=SerializePoseGraph.Request()
    request.filename=str(ROOT/"maps"/f"baseline{attempt}-{suffix}")
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=30)
    if not future.done():
        raise TimeoutError("graph serialization")
    response=future.result()
    if response.result!=response.RESULT_SUCCESS:
        raise RuntimeError(f"graph serialization result {response.result}")
    print("GRAPH_PRESERVED "+request.filename,flush=True)
try:
    if not client.wait_for_service(timeout_sec=10):
        raise TimeoutError("mapper serialization service")
    save("prefix")
    while not (ROOT/"driver-result.json").exists():
        if time.monotonic()>deadline:
            raise TimeoutError("bounded route-finish graph observer")
        rclpy.spin_once(node,timeout_sec=.05)
    save("final")
finally:
    node.destroy_node()
    rclpy.shutdown()
