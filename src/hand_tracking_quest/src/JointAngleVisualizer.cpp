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
#include <tuple>
#include <QList>

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
    // If the number of sliders doesn't match the number of angles, rebuild the layout.
    if (sliderWidgets_.size() != angles.size())
    {
      // Remove all existing widgets.
      QLayoutItem *child;
      while ((child = layout_->takeAt(0)) != nullptr) {
        if (child->widget()) {
          child->widget()->deleteLater();
        }
        delete child;
      }
      sliderWidgets_.clear();

      // For each joint angle, create a horizontal layout with a label, slider, and a value label.
      for (int i = 0; i < angles.size(); ++i) {
        QWidget* rowWidget = new QWidget(this);
        QHBoxLayout* hLayout = new QHBoxLayout(rowWidget);
        QLabel* jointLabel = new QLabel(QString("Joint %1:").arg(i), rowWidget);
        QSlider* slider = new QSlider(Qt::Horizontal, rowWidget);
        // Assume a range of -180 to 180 degrees, multiplied by 10 for precision.
        slider->setRange(-1800, 1800);
        slider->setEnabled(false); // Read-only slider for visualization.
        QLabel* valueLabel = new QLabel(rowWidget);
        hLayout->addWidget(jointLabel);
        hLayout->addWidget(slider);
        hLayout->addWidget(valueLabel);
        layout_->addWidget(rowWidget);
        sliderWidgets_.append(std::make_tuple(slider, valueLabel));
      }
    }

    // Update each slider with the new angle value.
    for (int i = 0; i < angles.size(); ++i) {
      float angle = angles[i];
      int sliderValue = static_cast<int>(angle * 10); // Convert float to int (1 unit = 0.1°).
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
