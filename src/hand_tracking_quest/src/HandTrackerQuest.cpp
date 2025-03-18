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
  HandTrackerQuestNode()
  : Node("tcp_receiver_node"), running_(true)
  {
    // Declare parameters
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
  // Thread that accepts TCP connections and reads messages.
  void receiverThread()
  {
    // Create TCP socket.
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
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

    if (listen(sockfd_, 5) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error listening on socket: %s", strerror(errno));
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Listening for TCP connections on port %d...", port_);

    while (running_) {
      sockaddr_in clientAddr;
      socklen_t addrLen = sizeof(clientAddr);
      int clientSock = accept(sockfd_, (struct sockaddr*)&clientAddr, &addrLen);
      if (clientSock < 0) {
        if (running_) {
          RCLCPP_ERROR(this->get_logger(), "Error accepting connection: %s", strerror(errno));
        }
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Accepted TCP connection from %s:%d",
                  inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));

      char buffer[4096];
      // Read data from the connected client.
      while (running_) {
        ssize_t bytesReceived = recv(clientSock, buffer, sizeof(buffer) - 1, 0);
        if (bytesReceived > 0) {
          buffer[bytesReceived] = '\0';
          std::string message(buffer);
          {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.push(message);
          }
          queue_cond_.notify_one();
        } else if (bytesReceived == 0) {
          // Connection closed by client.
          RCLCPP_INFO(this->get_logger(), "TCP connection closed by client.");
          break;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", strerror(errno));
          break;
        }
      }
      close(clientSock);
    }
  }

  // Thread that processes messages from the queue and publishes them.
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
  std::vector<float> parseJointAngles(const std::string &message)
  {
    std::vector<float> angles;
    std::istringstream stream(message);
    std::string line;

    while (std::getline(stream, line)) {
      auto pos = line.find(':');
      if (pos != std::string::npos) {
        std::string valueStr = line.substr(pos + 1);
        valueStr.erase(0, valueStr.find_first_not_of(" \t"));
        if (valueStr.empty() || (!std::isdigit(valueStr[0]) && valueStr[0] != '-' && valueStr[0] != '+')) {
          continue;
        }
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
  int port_;
  bool debug_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create node options with intra-process communications enabled.
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<HandTrackerQuestNode>(options);
  
  // Set thread priority if possible.
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
