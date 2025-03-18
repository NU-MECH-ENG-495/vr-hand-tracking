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

class HandTrackerQuestNode : public rclcpp::Node
{
public:
  // Constructor now accepts NodeOptions.
  explicit HandTrackerQuestNode(const rclcpp::NodeOptions & options)
  : Node("udp_receiver_node", options), running_(true)
  {
    // Declare parameters.
    this->declare_parameter("port", 9000);
    this->declare_parameter("debug", false);
    
    port_ = this->get_parameter("port").as_int();
    debug_ = this->get_parameter("debug").as_bool();
    
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("hand_joint_angles", 10);
    receiver_thread_ = std::thread(&HandTrackerQuestNode::receiverThread, this);
    processing_thread_ = std::thread(&HandTrackerQuestNode::processingThread, this);
  }

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
  // Thread that receives UDP messages and pushes them into a queue.
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

  // Thread that processes messages from the queue, extracts joint angles,
  // and publishes them as a Float32MultiArray.
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

  // Modified parser that splits the message by commas.
  std::vector<float> parseJointAngles(const std::string &message)
  {
    std::vector<float> angles;
    std::istringstream ss(message);
    std::string token;
    while (std::getline(ss, token, ',')) {
      // Trim leading and trailing whitespace.
      token.erase(0, token.find_first_not_of(" \t"));
      token.erase(token.find_last_not_of(" \t") + 1);

      // Skip tokens that are empty or contain a colon (likely headers).
      if (token.empty() || token.find(':') != std::string::npos)
        continue;

      try {
        float angle = std::stof(token);
        angles.push_back(angle);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse token: %s", token.c_str());
      }
    }
    return angles;
  }

  // Member variables.
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<HandTrackerQuestNode>(options);
  
  // Set thread priority if possible (may require root privileges)
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
