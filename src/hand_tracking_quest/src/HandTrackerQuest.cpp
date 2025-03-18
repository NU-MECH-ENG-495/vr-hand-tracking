#include "hand_tracking_quest/HandTrackerQuest.hpp"
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sstream>
#include <cctype>

HandTrackerQuestNode::HandTrackerQuestNode(const rclcpp::NodeOptions & options)
  : Node("udp_receiver_node", options), running_(true)
{
  this->declare_parameter("port", 9000);
  this->declare_parameter("debug", false);

  port_ = this->get_parameter("port").as_int();
  debug_ = this->get_parameter("debug").as_bool();

  publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("hand_joint_angles", 10);
  receiver_thread_ = std::thread(&HandTrackerQuestNode::receiverThread, this);
  processing_thread_ = std::thread(&HandTrackerQuestNode::processingThread, this);
}

HandTrackerQuestNode::~HandTrackerQuestNode()
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

void HandTrackerQuestNode::receiverThread()
{
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

  if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Error binding socket: %s", strerror(errno));
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port %d...", port_);
  char buffer[4096];
  while (running_) {
    sockaddr_in clientAddr;
    socklen_t addrLen = sizeof(clientAddr);
    ssize_t bytesReceived = recvfrom(sockfd_, buffer, sizeof(buffer) - 1, 0,
                                       (struct sockaddr *)&clientAddr, &addrLen);
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

void HandTrackerQuestNode::processingThread()
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

std::vector<float> HandTrackerQuestNode::parseJointAngles(const std::string & message)
{
  std::vector<float> angles;
  std::regex numberRegex("[-+]?[0-9]*\\.?[0-9]+");
  auto numbersBegin = std::sregex_iterator(message.begin(), message.end(), numberRegex);
  auto numbersEnd = std::sregex_iterator();
  for (std::sregex_iterator i = numbersBegin; i != numbersEnd; ++i) {
    std::smatch match = *i;
    try {
      float angle = std::stof(match.str());
      angles.push_back(angle);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse number: %s", match.str().c_str());
    }
  }
  return angles;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<HandTrackerQuestNode>(options);
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
