/**
 * @file JointAngleVisualizer.cpp
 * @brief ROS2 + Qt visualizer for displaying joint angles using sliders.
 *
 * This file contains the implementation of a ROS2 node combined with a Qt GUI. The node subscribes
 * to the "hand_joint_angles" topic and updates a set of sliders that visualize the joint angle values.
 */

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QTimer>
#include <QVector>
#include <QString>
#include <QDebug>
#include <QObject>
#include <QList>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

/**
 * @brief Structure to hold joint configuration data.
 *
 * This structure holds the name, minimum, and maximum range (in degrees) of a joint.
 */
struct JointConfig {
  QString name; ///< The human-readable joint name.
  int min;      ///< The minimum joint angle in degrees.
  int max;      ///< The maximum joint angle in degrees.
};

/**
 * @brief Joint configuration for each joint in the specified order.
 *
 * The vector contains configuration for each joint to be visualized.
 */
static const QVector<JointConfig> jointConfigs = {
  { "Thumb CMC Flexion",   -10, 50 },
  { "Thumb CMC Abduction", -50, -20},
  { "Thumb MCP Abduction", -10, 40 },
  { "Thumb MCP Flexion",   -40, 80 },
  { "Index MCP Flexion",   -20, 90 },
  { "Index MCP Abduction", -15, 15 },
  { "Index PIP",             0, 90 },
  { "Middle MCP Flexion",  -20, 90 },
  { "Middle MCP Abduction", 0 , 16 },
  { "Middle PIP",           0 , 90 },
  { "Ring MCP Flexion",    -20, 90 },
  { "Ring MCP Abduction",    8, 20 },
  { "Ring PIP",              0, 90 },
  { "Pinky MCP Flexion",   -20, 90 },
  { "Pinky MCP Abduction", -40, 15 },
  { "Pinky PIP",             0, 90 }
};

/**
 * @brief ROS2 node that subscribes to joint angle data and integrates with Qt.
 *
 * This class inherits from both QObject and rclcpp::Node to allow integration of ROS2 communication
 * with Qt signals and slots. It subscribes to the "hand_joint_angles" topic and emits a signal
 * when new joint angle data is received.
 */
class JointAnglesSubscriber : public QObject, public rclcpp::Node
{
  Q_OBJECT
public:
  /**
   * @brief Constructor.
   *
   * Creates the ROS2 node and initializes the subscription.
   *
   * @param options Node options.
   */
  explicit JointAnglesSubscriber(const rclcpp::NodeOptions & options)
  : rclcpp::Node("qt_joint_angles_subscriber", options)
  {
    // Create a subscription to the "hand_joint_angles" topic.
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "hand_joint_angles", 10,
      std::bind(&JointAnglesSubscriber::topic_callback, this, std::placeholders::_1));
  }

signals:
  /**
   * @brief Signal emitted when new joint angle data is received.
   *
   * @param angles A QVector containing the joint angle values.
   */
  void newJointAngles(const QVector<float>& angles);

private:
  /**
   * @brief Callback function for ROS2 subscription.
   *
   * Converts incoming ROS message containing joint angle data into a QVector<float> and emits
   * the newJointAngles signal.
   *
   * @param msg The incoming ROS message of type std_msgs::msg::Float32MultiArray.
   */
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Use the QVector constructor with iterators.
    QVector<float> angles(msg->data.begin(), msg->data.end());
    emit newJointAngles(angles);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_; ///< ROS2 subscription.
};

/**
 * @brief Main window class for visualizing joint angles using sliders.
 *
 * This class creates a Qt-based GUI that displays joint angles as read-only sliders. The sliders
 * are configured based on the joint configuration data and updated in real time as new data is received.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  /**
   * @brief Constructor.
   *
   * Sets up the main window, including the layout and slider widgets, and connects the ROS2
   * subscriber to the update mechanism.
   *
   * @param subscriber Pointer to the JointAnglesSubscriber.
   * @param executor Pointer to the ROS2 executor used to process callbacks.
   * @param parent Optional parent widget.
   */
  MainWindow(JointAnglesSubscriber* subscriber,
             rclcpp::executors::SingleThreadedExecutor* executor,
             QWidget* parent = nullptr)
    : QMainWindow(parent), subscriber_(subscriber), executor_(executor)
  {
    // Set up a vertical layout to hold the sliders.
    QWidget* centralWidget = new QWidget(this);
    layout_ = new QVBoxLayout(centralWidget);
    setCentralWidget(centralWidget);

    // Build slider widgets based on jointConfigs.
    rebuildSliderWidgets();

    // Connect the subscriber's signal to update the slider display.
    connect(subscriber_, &JointAnglesSubscriber::newJointAngles,
            this, &MainWindow::updateAngles);

    // Create a timer to periodically process incoming ROS messages.
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::spinOnce);
    timer->start(50);  // Process callbacks every 50 milliseconds.
  }

public slots:
  /**
   * @brief Update the slider widgets with the received joint angles.
   *
   * Updates each slider with the corresponding joint angle value. If the number of received angles
   * does not match the expected count, a warning is displayed.
   *
   * @param angles A QVector containing the joint angle values.
   */
  void updateAngles(const QVector<float>& angles)
  {
    // If the message doesn't match our expected joint count, warn.
    if (angles.size() != jointConfigs.size()) {
      qWarning() << "Expected" << jointConfigs.size() << "angles but received" << angles.size();
      return;
    }

    // Update each slider with the new angle value.
    // We multiply the angle by 10 for 0.1° resolution.
    for (int i = 0; i < angles.size(); ++i) {
      float angle = angles[i];
      int sliderValue = static_cast<int>(angle * 10);
      QSlider* slider = std::get<0>(sliderWidgets_[i]);
      QLabel* valueLabel = std::get<1>(sliderWidgets_[i]);
      slider->setValue(sliderValue);
      valueLabel->setText(QString::number(angle, 'f', 1));
    }
  }

  /**
   * @brief Process pending ROS callbacks.
   *
   * Calls spin_some() on the ROS2 executor to process incoming ROS messages.
   */
  void spinOnce()
  {
    executor_->spin_some();
  }

private:
  /**
   * @brief Rebuild the slider widgets based on the joint configuration.
   *
   * Clears any existing widgets from the layout and creates a new set of slider widgets
   * based on the global jointConfigs vector. Each slider is accompanied by a label displaying
   * the joint name and its current value.
   */
  void rebuildSliderWidgets()
  {
    // Clear any existing widgets from the layout.
    QLayoutItem *child;
    while ((child = layout_->takeAt(0)) != nullptr) {
      if (child->widget()) {
        child->widget()->deleteLater();
      }
      delete child;
    }
    sliderWidgets_.clear();

    // For each joint, create a horizontal row containing:
    // a label with the joint name, a read-only slider, and a value label.
    for (int i = 0; i < jointConfigs.size(); ++i) {
      const JointConfig& config = jointConfigs[i];
      QWidget* rowWidget = new QWidget(this);
      QHBoxLayout* hLayout = new QHBoxLayout(rowWidget);

      QLabel* jointLabel = new QLabel(config.name, rowWidget);
      QSlider* slider = new QSlider(Qt::Horizontal, rowWidget);
      // Set slider range (multiply by 10 for 0.1° resolution).
      slider->setRange(config.min * 10, config.max * 10);
      slider->setEnabled(false); // Read-only slider for visualization.
      QLabel* valueLabel = new QLabel(rowWidget);
      hLayout->addWidget(jointLabel);
      hLayout->addWidget(slider);
      hLayout->addWidget(valueLabel);
      layout_->addWidget(rowWidget);
      sliderWidgets_.append(std::make_tuple(slider, valueLabel));
    }
  }

  JointAnglesSubscriber* subscriber_;  ///< Pointer to the ROS2 subscriber.
  rclcpp::executors::SingleThreadedExecutor* executor_; ///< ROS2 executor for processing callbacks.
  QVBoxLayout* layout_;  ///< Layout for slider widgets.
  /// List of tuples containing pointers to a QSlider and its associated QLabel.
  QList<std::tuple<QSlider*, QLabel*>> sliderWidgets_;
};

/**
 * @brief Main entry point for the application.
 *
 * Initializes ROS2 and Qt, creates the ROS2 node and Qt main window, and enters the Qt event loop.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return int Application exit status.
 */
int main(int argc, char *argv[])
{
  // Initialize ROS2.
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto subscriber = new JointAnglesSubscriber(options);

  // Create a single-threaded executor and add our node.
  auto executor = new rclcpp::executors::SingleThreadedExecutor();
  executor->add_node(subscriber->get_node_base_interface());

  // Initialize the Qt application.
  QApplication app(argc, argv);
  MainWindow window(subscriber, executor);
  window.show();

  int result = app.exec();

  // Clean up.
  rclcpp::shutdown();
  delete executor;
  delete subscriber;
  return result;
}

#include "JointAngleVisualizer.moc"
