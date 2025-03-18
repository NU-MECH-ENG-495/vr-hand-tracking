/**
 * @file HandTrackerQuestNode.cpp
 * @brief ROS2 node that receives UDP packets, parses joint angles using regex, and publishes them.
 *
 * This file implements a ROS2 node that creates a UDP socket to receive joint angle data,
 * uses a regular expression to extract floating-point numbers from the incoming message,
 * and publishes the parsed joint angles as a std_msgs::msg::Float32MultiArray.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <cctype>
#include <regex>

/**
 * @brief Class that implements a UDP receiver node for joint angles.
 *
 * This node creates a UDP socket on the specified port, receives data packets,
 * parses the joint angles using a regular expression, and publishes them.
 */
class HandTrackerQuestNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor.
   *
   * Initializes the node with the given options, declares parameters, and starts
   * the receiver and processing threads.
   *
   * @param options ROS2 node options.
   */
  explicit HandTrackerQuestNode(const rclcpp::NodeOptions & options)
  : Node("udp_receiver_node", options), running_(true)
  {
    // Declare parameters.
    this->declare_parameter("port", 9000);
    this->declare_parameter("debug", false);
    
    port_ = this->get_parameter("port").as_int();
    debug_ = this->get_parameter("debug").as_bool();
    
    // Create the publisher for joint angle messages.
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("hand_joint_angles", 10);
    // Launch the receiver and processing threads.
    receiver_thread_ = std::thread(&HandTrackerQuestNode::receiverThread, this);
    processing_thread_ = std::thread(&HandTrackerQuestNode::processingThread, this);
  }

  /**
   * @brief Destructor.
   *
   * Stops the threads and closes the socket.
   */
  ~HandTrackerQuestNode()
  {
    running_ = false;
    
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
    
    if(receiver_thread_.joinable())
      receiver_thread_.join();
    if(processing_thread_.joinable())
      processing_thread_.join();
  }

private:
  /**
   * @brief Thread function to receive UDP messages.
   *
   * Creates a UDP socket, binds to the specified port, and listens for incoming messages.
   * Received messages are pushed into a queue for processing.
   */
  void receiverThread()
  {
    // Create UDP socket.
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error creating socket: %s", strerror(errno));
      return;
    }

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error binding socket: %s", strerror(errno));
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", port_);

    char buffer[4096];
    while (running_) {
      sockaddr_in clientAddr;
      socklen_t addrLen = sizeof(clientAddr);
      ssize_t bytesReceived = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                                         (struct sockaddr*)&clientAddr, &addrLen);
      if (bytesReceived > 0) {
        buffer[bytesReceived] = '\0';
        std::string message(buffer);
        RCLCPP_INFO(this->get_logger(), "Received UDP message: %s", message.c_str());
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          message_queue_.push(message);
        }
        queue_cond_.notify_one();
      }
    }
  }

  /**
   * @brief Thread function to process received messages.
   *
   * Waits for messages to appear in the queue, then processes each message by parsing
   * joint angles and publishing them as a std_msgs::msg::Float32MultiArray.
   */
  void processingThread()
  {
    while (running_) {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cond_.wait(lock, [this]() { return !message_queue_.empty() || !running_; });
      while (!message_queue_.empty()) {
        std::string message = message_queue_.front();
        message_queue_.pop();
        lock.unlock();

        RCLCPP_INFO(this->get_logger(), "Processing message: %s", message.c_str());

        std::vector<float> angles = parseJointAngles(message);
        if (!angles.empty()) {
          std_msgs::msg::Float32MultiArray msg;
          msg.data = angles;
          publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Published %zu joint angles", angles.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "No valid joint angles found in the message.");
        }
        lock.lock();
      }
    }
  }

  /**
   * @brief Parses a message string to extract joint angle values.
   *
   * Uses a regular expression to search for all floating-point numbers in the message string.
   *
   * @param message The input message containing joint angle data.
   * @return std::vector<float> A vector of parsed joint angle values.
   */
  std::vector<float> parseJointAngles(const std::string &message)
  {
    std::vector<float> angles;
    // Regular expression to match signed numbers with optional decimals.
    std::regex numberRegex("[-+]?[0-9]*\\.?[0-9]+");
    auto numbersBegin = std::sregex_iterator(message.begin(), message.end(), numberRegex);
    auto numbersEnd = std::sregex_iterator();

    for (std::sregex_iterator i = numbersBegin; i != numbersEnd; ++i) {
      std::smatch match = *i;
      try {
        float angle = std::stof(match.str());
        angles.push_back(angle);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse number: %s", match.str().c_str());
      }
    }
    return angles;
  }

  // Member variables.
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_; ///< ROS2 publisher for joint angles.
  std::thread receiver_thread_;         ///< Thread for receiving UDP messages.
  std::thread processing_thread_;       ///< Thread for processing and publishing messages.
  std::queue<std::string> message_queue_; ///< Queue to hold received messages.
  std::mutex queue_mutex_;              ///< Mutex for protecting access to the message queue.
  std::condition_variable queue_cond_;  ///< Condition variable for signaling new messages.
  int sockfd_{-1};                      ///< Socket file descriptor.
  bool running_;                        ///< Flag to control the threads.
  int port_;                            ///< UDP port to bind.
  bool debug_;                          ///< Debug flag.
};

/**
 * @brief Main entry point for the UDP receiver node.
 *
 * Initializes ROS2, creates the HandTrackerQuestNode, adjusts thread priority if possible,
 * and enters the ROS2 spin loop.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Exit status.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<HandTrackerQuestNode>(options);
  
  // Set thread priority if possible (may require root privileges).
  pthread_t this_thread = pthread_self();
  struct sched_param params;
  params.sched_priority = sched_get_priority_max(SCHED_FIFO);
  if (pthread_setschedparam(this_thread, SCHED_FIFO, &params) != 0) {
    RCLCPP_WARN(node->get_logger(), "Could not set thread priority: %s", strerror(errno));
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
