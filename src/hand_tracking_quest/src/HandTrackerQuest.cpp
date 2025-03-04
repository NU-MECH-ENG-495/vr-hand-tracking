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
#include <fcntl.h>

class HandTrackerQuestNode : public rclcpp::Node
{
public:
  // Updated constructor to accept NodeOptions
  HandTrackerQuestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("udp_receiver_node", options), running_(true), received_count_(0), processed_count_(0)
  {
    // Declare parameters
    this->declare_parameter("port", 9000);
    this->declare_parameter("debug", false);
    
    int port = this->get_parameter("port").as_int();
    debug_ = this->get_parameter("debug").as_bool();
    
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("hand_joint_angles", 10);
    
    // Pre-allocate message
    joint_angles_msg_.data.reserve(50);
    
    // Set up socket
    if (!setupSocket(port)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set up socket, exiting");
      return;
    }
    
    // Single thread design - use a timer for processing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  // Try to process at 1000Hz to not miss packets
      std::bind(&HandTrackerQuestNode::processPackets, this));
    
    // Add a timer for periodic statistics logging
    stats_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&HandTrackerQuestNode::logStats, this));
  }

  ~HandTrackerQuestNode()
  {
    running_ = false;
    
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
  }

private:
  // Set up socket with non-blocking mode
  bool setupSocket(int port)
  {
    // Create UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error creating socket: %s", strerror(errno));
      return false;
    }

    // Set socket to non-blocking mode
    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    // Increase receive buffer size
    int rcvbuf = 2 * 1024 * 1024;  // 2MB buffer
    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to set receive buffer size: %s", strerror(errno));
    }
    
    // Enable socket reuse
    int reuse = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
    }

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error binding socket: %s", strerror(errno));
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", port);
    return true;
  }

  // Process incoming UDP packets directly without queuing
  void processPackets()
  {
    if (sockfd_ < 0) {
      return;
    }

    // Try to process multiple packets per timer callback
    for (int i = 0; i < 10; i++) {
      char buffer[8192];  // Larger buffer for bigger packets
      sockaddr_in clientAddr;
      socklen_t addrLen = sizeof(clientAddr);
      
      ssize_t bytesReceived = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                                    (struct sockaddr*)&clientAddr, &addrLen);
                                    
      // Break if no more packets or error
      if (bytesReceived <= 0) {
        if (bytesReceived < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(this->get_logger(), "Error receiving UDP packet: %s", strerror(errno));
        }
        break;
      }
      
      // Null-terminate the buffer
      buffer[bytesReceived] = '\0';
      received_count_++;
      
      if (debug_) {
        RCLCPP_DEBUG(this->get_logger(), "Received %zd bytes", bytesReceived);
      }
      
      // Process this packet immediately
      std::vector<float> angles = parseJointAngles(buffer, bytesReceived);
      if (!angles.empty()) {
        // Publish immediately
        joint_angles_msg_.data = std::move(angles);
        publisher_->publish(joint_angles_msg_);
        processed_count_++;
      }
    }
  }

  // Optimized parsing function that works directly on the buffer
  std::vector<float> parseJointAngles(const char* buffer, size_t length)
  {
    std::vector<float> angles;
    angles.reserve(50);
    
    size_t pos = 0;
    
    while (pos < length) {
      // Find the end of current line
      size_t lineEnd = pos;
      while (lineEnd < length && buffer[lineEnd] != '\n') {
        lineEnd++;
      }
      
      // Find the colon in the current line
      size_t colonPos = pos;
      while (colonPos < lineEnd && buffer[colonPos] != ':') {
        colonPos++;
      }
      
      if (colonPos < lineEnd) {
        // Extract value after colon
        size_t valueStart = colonPos + 1;
        while (valueStart < lineEnd && (buffer[valueStart] == ' ' || buffer[valueStart] == '\t')) {
          valueStart++;
        }
        
        // Check if the first character is a digit or sign
        if (valueStart < lineEnd && 
            (std::isdigit(buffer[valueStart]) || buffer[valueStart] == '-' || buffer[valueStart] == '+')) {
          
          // Find the end of the number (before any degree symbol)
          size_t valueEnd = valueStart;
          while (valueEnd < lineEnd && 
                (std::isdigit(buffer[valueEnd]) || buffer[valueEnd] == '.' || 
                 buffer[valueEnd] == '-' || buffer[valueEnd] == '+' || buffer[valueEnd] == 'e' || 
                 buffer[valueEnd] == 'E')) {
            valueEnd++;
          }
          
          // Convert to float using a temporary null-terminated string
          if (valueEnd > valueStart) {
            char temp[32] = {0};  // Temporary buffer for the number
            size_t len = valueEnd - valueStart;
            if (len < sizeof(temp) - 1) {
              memcpy(temp, buffer + valueStart, len);
              temp[len] = '\0';
              try {
                float angle = std::strtof(temp, nullptr);
                angles.push_back(angle);
              } catch (...) {
                // Ignore parsing errors
              }
            }
          }
        }
      }
      
      // Move to the next line
      pos = lineEnd + 1;
      if (lineEnd >= length) {
        break;
      }
    }
    
    return angles;
  }
  
  // Log statistics
  void logStats()
  {
    static int last_received = 0;
    static int last_processed = 0;
    static auto last_time = std::chrono::steady_clock::now();
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() / 1000.0;
    
    int received_diff = received_count_ - last_received;
    int processed_diff = processed_count_ - last_processed;
    
    double received_rate = received_diff / elapsed_seconds;
    double processed_rate = processed_diff / elapsed_seconds;
    
    RCLCPP_INFO(this->get_logger(), 
      "Stats: Received rate: %.1f msg/s, Processed rate: %.1f msg/s", 
      received_rate, processed_rate);
      
    last_received = received_count_;
    last_processed = processed_count_;
    last_time = now;
    
    // Debugging - check if we're receiving any data
    if (received_diff == 0) {
      RCLCPP_WARN(this->get_logger(), "No packets received in the last %.1f seconds!", elapsed_seconds);
    }
  }

  // Member variables
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std_msgs::msg::Float32MultiArray joint_angles_msg_;  // Pre-allocated message
  int sockfd_{-1};
  std::atomic<bool> running_;
  std::atomic<int> received_count_;
  std::atomic<int> processed_count_;
  bool debug_;
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