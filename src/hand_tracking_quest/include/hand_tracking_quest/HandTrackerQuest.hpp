#ifndef HAND_TRACKER_QUEST_HPP
#define HAND_TRACKER_QUEST_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <regex>
#include <string>

/**
 * @brief ROS2 node that receives UDP packets, parses joint angles using regex, and publishes them.
 */
class HandTrackerQuestNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor.
   *
   * @param options ROS2 node options.
   */
  explicit HandTrackerQuestNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Destructor.
   */
  ~HandTrackerQuestNode();

  // Option 1: Make the parser public for testing.
  std::vector<float> parseJointAngles(const std::string & message);

  // Alternatively, if you prefer to keep it private, add the following friend declaration:
  // friend class HandTrackerQuestNodeTest;

private:
  void receiverThread();
  void processingThread();

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  std::thread receiver_thread_;
  std::thread processing_thread_;
  std::queue<std::string> message_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_;
  int sockfd_{-1};
  bool running_;
  int port_;
  bool debug_;
};

#endif // HAND_TRACKER_QUEST_HPP
