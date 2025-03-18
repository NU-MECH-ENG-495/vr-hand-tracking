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
#include <cctype>

class HandTrackerQuestNode : public rclcpp::Node
{
public:
  HandTrackerQuestNode()
  : Node("udp_receiver_node"), running_(true)
  {
    // Declare parameters
    this->declare_parameter("port", 9000);
    this->declare_parameter("debug", false);
    
    int port = this->get_parameter("port").as_int();
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
  }

private:
  // Thread that receives UDP messages and pushes them into a queue.
  void receiverThread()
  {
    // Create UDP socket.
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error creating socket: %s", strerror(errno));
      return false;
    }

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error binding socket");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port 9000...");

    char buffer[4096];
    while (running_) {
      sockaddr_in clientAddr;
      socklen_t addrLen = sizeof(clientAddr);
      ssize_t bytesReceived = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                                         (struct sockaddr*)&clientAddr, &addrLen);
      if (bytesReceived > 0) {
        buffer[bytesReceived] = '\0';
        std::string message(buffer);
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

        std::vector<float> angles = parseJointAngles(message);
        if (!angles.empty()) {
          std_msgs::msg::Float32MultiArray msg;
          msg.data = angles;
          publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Published %zu joint angles", angles.size());
        }
        lock.lock();
      }
    }
  }

  // Parses the incoming message to extract joint angle values.
  // It looks for lines with a colon and checks if the value part starts with a digit or a sign.
  std::vector<float> parseJointAngles(const std::string &message)
  {
    std::vector<float> angles;
    std::istringstream stream(message);
    std::string line;

    // Process each line of the message.
    while (std::getline(stream, line)) {
      auto pos = line.find(':');
      if (pos != std::string::npos) {
        std::string valueStr = line.substr(pos + 1);
        // Trim whitespace.
        valueStr.erase(0, valueStr.find_first_not_of(" \t"));
        // Check if the first character is a digit or a sign.
        if (valueStr.empty() || (!std::isdigit(valueStr[0]) && valueStr[0] != '-' && valueStr[0] != '+')) {
          continue;
        }
        // Remove any trailing degree symbol.
        size_t degreePos = valueStr.find("°");
        if (degreePos != std::string::npos) {
          valueStr = valueStr.substr(0, degreePos);
        }
        try {
          float angle = std::stof(valueStr);
          angles.push_back(angle);
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse angle from line: %s", line.c_str());
        }
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create node options with intra-process comms enabled
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
