#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <cctype>
#include <fcntl.h>

class HandTrackerQuestNode : public rclcpp::Node
{
public:
  HandTrackerQuestNode()
  : Node("udp_receiver_node"), running_(true), received_count_(0), processed_count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("hand_joint_angles", 10);
    
    // Initialize the pre-allocated message
    joint_angles_msg_.data.reserve(50);  // Pre-allocate space for expected number of angles
    
    // Create and configure the socket first to avoid race conditions
    setupSocket();
    
    // Start threads only after socket is set up
    receiver_thread_ = std::thread(&HandTrackerQuestNode::receiverThread, this);
    processing_thread_ = std::thread(&HandTrackerQuestNode::processingThread, this);
    
    // Add a timer for periodic statistics logging
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&HandTrackerQuestNode::logStats, this));
  }

  ~HandTrackerQuestNode()
  {
    running_ = false;
    queue_cond_.notify_all();

    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
  }

private:
  // Set up socket with non-blocking mode and proper timeout
  void setupSocket()
  {
    // Create UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error creating socket");
      return;
    }

    // Set socket to non-blocking mode
    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms timeout
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Set receive buffer size
    int rcvbuf = 1024 * 1024;  // 1MB buffer
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9000);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error binding socket");
      close(sockfd_);
      sockfd_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port 9000...");
  }

  // Thread that receives UDP messages and pushes them into a queue
  void receiverThread()
  {
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot start receiver thread: invalid socket");
      return;
    }

    char buffer[4096];
    sockaddr_in clientAddr;
    socklen_t addrLen = sizeof(clientAddr);
    
    const size_t MAX_QUEUE_SIZE = 100;  // Limit queue size to prevent memory issues

    while (running_) {
      ssize_t bytesReceived = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                                      (struct sockaddr*)&clientAddr, &addrLen);
                                      
      if (bytesReceived > 0) {
        buffer[bytesReceived] = '\0';
        
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          // Check queue size and potentially drop messages if we're falling behind
          if (message_queue_.size() < MAX_QUEUE_SIZE) {
            // Use emplace to avoid extra copy
            message_queue_.emplace(buffer, bytesReceived);
            received_count_++;
            queue_cond_.notify_one();
          } else {
            // We're falling behind, log a warning periodically
            static int drop_count = 0;
            if (++drop_count % 100 == 0) {
              RCLCPP_WARN(this->get_logger(), "Queue full, dropped %d messages", drop_count);
            }
          }
        }
      } else if (bytesReceived < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        // Only log real errors, not expected timeouts
        RCLCPP_ERROR(this->get_logger(), "Error receiving UDP packet: %s", strerror(errno));
      }
      
      // Small sleep to prevent CPU spinning in non-blocking mode
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Thread that processes messages from the queue
  void processingThread()
  {
    std::vector<char> buffer;
    buffer.reserve(4096);  // Pre-allocate to avoid resizing
    
    while (running_) {
      std::string message;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cond_.wait_for(lock, std::chrono::milliseconds(100), 
                            [this]() { return !message_queue_.empty() || !running_; });
        
        if (!running_) {
          break;
        }
        
        if (message_queue_.empty()) {
          continue;
        }
        
        // Process one message at a time to reduce lock contention
        message = std::move(message_queue_.front());
        message_queue_.pop();
      }
      
      std::vector<float> angles = parseJointAngles(message);
      if (!angles.empty()) {
        // Reuse the pre-allocated message
        joint_angles_msg_.data = std::move(angles);
        publisher_->publish(joint_angles_msg_);
        processed_count_++;
      }
    }
  }

  // Faster parsing function that avoids excessive string operations
  std::vector<float> parseJointAngles(const std::string &message)
  {
    std::vector<float> angles;
    angles.reserve(50);  // Pre-allocate expected number of angles
    
    size_t pos = 0;
    // size_t lineStart = 0;
    
    while (pos < message.size()) {
      // Find the end of current line
      size_t lineEnd = message.find('\n', pos);
      if (lineEnd == std::string::npos) {
        lineEnd = message.size();
      }
      
      // Find the colon in the current line
      size_t colonPos = message.find(':', pos);
      if (colonPos != std::string::npos && colonPos < lineEnd) {
        // Extract value after colon
        size_t valueStart = colonPos + 1;
        while (valueStart < lineEnd && (message[valueStart] == ' ' || message[valueStart] == '\t')) {
          valueStart++;
        }
        
        // Check if the first character is a digit or sign
        if (valueStart < lineEnd && 
            (std::isdigit(message[valueStart]) || message[valueStart] == '-' || message[valueStart] == '+')) {
          
          // Find the end of the number (before any degree symbol)
          size_t valueEnd = valueStart;
          while (valueEnd < lineEnd && 
                (std::isdigit(message[valueEnd]) || message[valueEnd] == '.' || 
                 message[valueEnd] == '-' || message[valueEnd] == '+' || message[valueEnd] == 'e' || 
                 message[valueEnd] == 'E')) {
            valueEnd++;
          }
          
          // Convert substring to float
          try {
            float angle = std::stof(message.substr(valueStart, valueEnd - valueStart));
            angles.push_back(angle);
          } catch (...) {
            // Silently ignore parsing errors to avoid logging overhead
          }
        }
      }
      
      // Move to the next line
      pos = lineEnd + 1;
    }
    
    return angles;
  }
  
  // Periodic logging of statistics to monitor performance
  void logStats()
  {
    static int last_received = 0;
    static int last_processed = 0;
    
    int received_diff = received_count_ - last_received;
    int processed_diff = processed_count_ - last_processed;
    
    RCLCPP_INFO(this->get_logger(), 
      "Stats: Received rate: %d msg/s, Processed rate: %d msg/s, Queue size: %zu", 
      received_diff / 5, processed_diff / 5, message_queue_.size());
      
    last_received = received_count_;
    last_processed = processed_count_;
  }

  // Member variables
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  std::thread receiver_thread_;
  std::thread processing_thread_;
  std::queue<std::string> message_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_;
  int sockfd_{-1};
  bool running_;
  std_msgs::msg::Float32MultiArray joint_angles_msg_;  // Pre-allocated message
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::atomic<int> received_count_;
  std::atomic<int> processed_count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandTrackerQuestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}