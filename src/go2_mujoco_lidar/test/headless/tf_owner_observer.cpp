// W03 TF edge + publisher-GID tap. Humble rclpy discards publisher identity;
// rclcpp MessageInfo preserves it. Cyclone's MessageInfo stores a publication
// handle, while graph endpoints use GUIDs. Public DDS built-in publication
// metadata maps those identities; TSV is consumed by verify_bringup.py.
// This read-only observer owns no TF/clock edge and accepts use_sim_time=true.
// Optional G03 command tap preserves the same RMW handle mapping for each Twist.
// Steady receipt time measures the bridge watchdog window; it is not send time.
// Metadata is consumed in batches, never capped at 256 publication instances.
// Action status snapshots form an independent diagnostic channel, not a GID fallback.
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <dds/dds.h>

#include <array>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <vector>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("w03_tf_owner_observer");
  // Reuse the domain RMW already configured with localhost + the W03 tag.
  const auto participant = dds_create_participant(41, nullptr, nullptr);
  const auto publications = dds_create_reader(
    participant, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr);
  if (participant < 0 || publications < 0) {
    throw std::runtime_error("cannot observe DDS publication metadata");
  }
  std::set<dds_instance_handle_t> known;
  const bool observe_commands = node->declare_parameter("observe_commands", false);
  std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> actions;
  auto metadata = node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
    // dds_read repeatedly returned the oldest 256 valid instances. Taking and
    // returning every batch lets late-created publishers reach the identity map.
    while (true) {
      std::array<void *, 256> samples{};
      std::array<dds_sample_info_t, 256> infos{};
      const auto count = dds_take(
        publications, samples.data(), infos.data(), samples.size(), samples.size());
      if (count < 0) {
        throw std::runtime_error("DDS publication metadata take failed");
      }
      if (count == 0) {break;}
      for (int index = 0; index < count; ++index) {
        if (!infos[index].valid_data || !known.insert(infos[index].instance_handle).second) {
          continue;
        }
        const auto * endpoint = static_cast<dds_builtintopic_endpoint_t *>(samples[index]);
        std::ostringstream handle;
        handle << std::hex << std::setfill('0');
        const auto * bytes = reinterpret_cast<const unsigned char *>(&infos[index].instance_handle);
        for (std::size_t byte = 0; byte < RMW_GID_STORAGE_SIZE; ++byte) {
          handle << std::setw(2) << (byte < sizeof(dds_instance_handle_t) ? bytes[byte] : 0U);
        }
        std::ostringstream guid;
        guid << std::hex << std::setfill('0');
        for (std::size_t byte = 0; byte < RMW_GID_STORAGE_SIZE; ++byte) {
          guid << std::setw(2) << (byte < sizeof(endpoint->key.v) ? endpoint->key.v[byte] : 0U);
        }
        std::cout << "GID_MAP\t" << handle.str() << '\t' << guid.str() << std::endl;
      }
      dds_return_loan(publications, samples.data(), count);
    }
    if (observe_commands) {
      for (const auto & [topic, types] : node->get_topic_names_and_types()) {
        if (topic.size() < 15 || topic.substr(topic.size() - 15) != "/_action/status" ||
          actions.count(topic) || std::find(types.begin(), types.end(),
          "action_msgs/msg/GoalStatusArray") == types.end()) {continue;}
        std::cout << "ACTION_TOPIC\t" << topic;
        for (const auto & endpoint : node->get_publishers_info_by_topic(topic)) {
          std::cout << '\t' << endpoint.node_namespace() << '/' << endpoint.node_name();
        }
        std::cout << std::endl;
        actions[topic] = node->create_subscription<action_msgs::msg::GoalStatusArray>(
          topic, rclcpp::QoS(100).transient_local(),
          [topic](action_msgs::msg::GoalStatusArray::ConstSharedPtr message) {
            const auto received = std::chrono::steady_clock::now().time_since_epoch();
            std::cout << "ACTION_STATUS\t"
                      << std::chrono::duration_cast<std::chrono::nanoseconds>(received).count()
                      << '\t' << topic << '\t';
            bool first = true;
            for (const auto & status : message->status_list) {
              if (!first) {std::cout << ',';}
              first = false;
              std::ostringstream uuid;
              uuid << std::hex << std::setfill('0');
              for (const auto byte : status.goal_info.goal_id.uuid) {
                uuid << std::setw(2) << static_cast<unsigned int>(byte);
              }
              std::cout << uuid.str() << ':' << static_cast<int>(status.status);
            }
            std::cout << std::endl;
          });
      }
    }
    });
  const auto receive = [](
    tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info)
    {
      std::ostringstream gid;
      gid << std::hex << std::setfill('0');
      for (const auto byte : info.get_rmw_message_info().publisher_gid.data) {
        gid << std::setw(2) << static_cast<unsigned int>(byte);
      }
      for (const auto & transform : message->transforms) {
        std::cout << "TF_OWNER\t" << transform.header.frame_id << '\t'
                  << transform.child_frame_id << '\t' << gid.str() << std::endl;
      }
    };
  auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100), receive);
  auto fixed = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local(), receive);
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> commands;
  if (observe_commands) {
    for (const std::string topic : {"/go2_nav2/shadow_cmd_vel", "/cmd_vel"}) {
      commands.push_back(node->create_subscription<geometry_msgs::msg::Twist>(
        topic, rclcpp::QoS(1000),
        [node, topic](geometry_msgs::msg::Twist::ConstSharedPtr message,
                      const rclcpp::MessageInfo & info) {
          const auto received = std::chrono::steady_clock::now().time_since_epoch();
          std::ostringstream gid;
          gid << std::hex << std::setfill('0');
          for (const auto byte : info.get_rmw_message_info().publisher_gid.data)
            gid << std::setw(2) << static_cast<unsigned int>(byte);
          std::cout << std::setprecision(17) << "COMMAND_OWNER\t"
                    << std::chrono::duration_cast<std::chrono::nanoseconds>(received).count()
                    << '\t' << node->get_clock()->now().nanoseconds() << '\t'
                    << topic << '\t' << gid.str() << '\t'
                    << message->linear.x << '\t' << message->angular.z << '\t'
                    << message->linear.y << '\t' << message->linear.z << '\t'
                    << message->angular.x << '\t' << message->angular.y << std::endl;
        }));
    }
  }
  rclcpp::spin(node);
  dds_delete(participant);
  rclcpp::shutdown();
  return 0;
}
