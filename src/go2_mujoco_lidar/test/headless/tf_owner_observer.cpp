// W03 TF edge + publisher-GID tap. Humble rclpy discards publisher identity;
// rclcpp MessageInfo preserves it. Cyclone's MessageInfo stores a publication
// handle, while graph endpoints use GUIDs. Public DDS built-in publication
// metadata maps those identities; TSV is consumed by verify_bringup.py.
// This read-only observer owns no TF/clock edge and accepts use_sim_time=true.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <dds/dds.h>

#include <array>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>

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
  auto metadata = node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
      std::array<void *, 256> samples{};
      std::array<dds_sample_info_t, 256> infos{};
      const auto count = dds_read(
        publications, samples.data(), infos.data(), samples.size(), samples.size());
      if (count < 0) {
        throw std::runtime_error("DDS publication metadata read failed");
      }
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
  rclcpp::spin(node);
  dds_delete(participant);
  rclcpp::shutdown();
  return 0;
}
