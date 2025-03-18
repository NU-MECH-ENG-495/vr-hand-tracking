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

// Define a struct to hold joint configuration.
struct JointConfig {
  QString name;
  int min; // in degrees
  int max; // in degrees
};

// Joint configuration for each joint in the specified order.
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

// This class combines a ROS2 node with QObject to allow Qt signals/slots.
class JointAnglesSubscriber : public QObject, public rclcpp::Node
{
  Q_OBJECT
public:
  explicit JointAnglesSubscriber(const rclcpp::NodeOptions & options)
  : rclcpp::Node("qt_joint_angles_subscriber", options)
  {
    // Create a subscription to the "hand_joint_angles" topic.
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "hand_joint_angles", 10,
      std::bind(&JointAnglesSubscriber::topic_callback, this, std::placeholders::_1));
  }

signals:
  // Signal to forward the joint angles to the GUI.
  void newJointAngles(const QVector<float>& angles);

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Use the QVector constructor with iterators.
    QVector<float> angles(msg->data.begin(), msg->data.end());
    emit newJointAngles(angles);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

// Main window that displays the joint angles as sliders.
class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
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
  // Update the slider widgets with the received joint angles.
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

  // Process pending ROS callbacks.
  void spinOnce()
  {
    executor_->spin_some();
  }

private:
  // Create slider widgets based on jointConfigs.
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

  JointAnglesSubscriber* subscriber_;
  rclcpp::executors::SingleThreadedExecutor* executor_;
  QVBoxLayout* layout_;
  // Each tuple contains a pointer to a QSlider and its associated QLabel.
  QList<std::tuple<QSlider*, QLabel*>> sliderWidgets_;
};

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
