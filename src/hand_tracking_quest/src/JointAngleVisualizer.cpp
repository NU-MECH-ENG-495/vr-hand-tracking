#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QVector>
#include <QString>
#include <QDebug>
#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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
    // Use the QVector constructor with iterators (avoids deprecated fromStdVector).
    QVector<float> angles(msg->data.begin(), msg->data.end());
    emit newJointAngles(angles);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

// Main window that displays the joint angles.
class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  MainWindow(JointAnglesSubscriber* subscriber,
             rclcpp::executors::SingleThreadedExecutor* executor,
             QWidget* parent = nullptr)
    : QMainWindow(parent), subscriber_(subscriber), executor_(executor)
  {
    // Set up a simple UI with a label.
    QWidget* centralWidget = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(centralWidget);
    label_ = new QLabel("Waiting for joint angles...", centralWidget);
    layout->addWidget(label_);
    setCentralWidget(centralWidget);

    // Connect the subscriber's signal to update the display.
    connect(subscriber_, &JointAnglesSubscriber::newJointAngles,
            this, &MainWindow::updateAngles);

    // Create a timer to periodically process incoming ROS messages.
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::spinOnce);
    timer->start(50);  // Spin every 50 milliseconds.
  }

public slots:
  // Update the label with the received joint angles.
  void updateAngles(const QVector<float>& angles)
  {
    QString text = "Joint Angles:\n";
    for (int i = 0; i < angles.size(); i++) {
      text += QString("Angle %1: %2\n").arg(i).arg(angles[i]);
    }
    label_->setText(text);
  }

  // Process pending ROS callbacks.
  void spinOnce()
  {
    executor_->spin_some();
  }

private:
  JointAnglesSubscriber* subscriber_;
  rclcpp::executors::SingleThreadedExecutor* executor_;
  QLabel* label_;
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
    