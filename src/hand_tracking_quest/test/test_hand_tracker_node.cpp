/**
 * @file test_hand_tracker_node.cpp
 * @brief Test suite for HandTrackerQuestNode's parser function using Google Test.
 */

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "hand_tracking_quest/HandTrackerQuest.hpp"

/**
 * @brief Test fixture for HandTrackerQuestNode.
 *
 * Provides access to the parseJointAngles() function.
 */
class HandTrackerQuestNodeTest : public ::testing::Test {
protected:
  // Define a type alias for the parse function pointer.
  typedef std::vector<float> (HandTrackerQuestNode::*ParseFunc)(const std::string &);

  rclcpp::NodeOptions options;
  std::shared_ptr<HandTrackerQuestNode> node;
  ParseFunc parseFunc;

  virtual void SetUp() {
    options.use_intra_process_comms(true);
    node = std::make_shared<HandTrackerQuestNode>(options);
    // Ensure parseJointAngles is accessible (it must be public or your test declared as friend)
    parseFunc = &HandTrackerQuestNode::parseJointAngles;
  }

  virtual void TearDown() {
    node.reset();
  }
};

/**
 * @brief Test that a valid message returns the expected joint angles.
 */
TEST_F(HandTrackerQuestNodeTest, ValidMessageTest) {
  std::string message =
    "Right hand:, 12.8, -45.9, 48.3, -4.2, 23.7, 6.1, 34.2, 28.0, 9.6, 41.8, 29.7, 13.6, 49.1, 16.3, 1.6, 43.2";
  std::vector<float> result = (node.get()->*parseFunc)(message);
  std::vector<float> expected = {12.8f, -45.9f, 48.3f, -4.2f, 23.7f, 6.1f, 34.2f, 28.0f,
    9.6f, 41.8f, 29.7f, 13.6f, 49.1f, 16.3f, 1.6f, 43.2f};
  EXPECT_EQ(result.size(), expected.size());
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(result[i], expected[i], 0.001);
  }
}

/**
 * @brief Test that an empty message returns an empty vector.
 */
TEST_F(HandTrackerQuestNodeTest, EmptyMessageTest) {
  std::string message = "";
  std::vector<float> result = (node.get()->*parseFunc)(message);
  EXPECT_TRUE(result.empty());
}

/**
 * @brief Test that a message with no numbers returns an empty vector.
 */
TEST_F(HandTrackerQuestNodeTest, NoNumbersTest) {
  std::string message = "No numbers here!";
  std::vector<float> result = (node.get()->*parseFunc)(message);
  EXPECT_TRUE(result.empty());
}

/**
 * @brief Test that a mixed text message returns only the valid numbers.
 */
TEST_F(HandTrackerQuestNodeTest, MixedTextTest) {
  std::string message = "Angle: +30.5 degrees and -20.3";
  std::vector<float> result = (node.get()->*parseFunc)(message);
  std::vector<float> expected = {30.5f, -20.3f};
  EXPECT_EQ(result.size(), expected.size());
  for (size_t i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(result[i], expected[i], 0.001);
  }
}

/**
 * @brief Main entry point for the test executable.
 */
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
