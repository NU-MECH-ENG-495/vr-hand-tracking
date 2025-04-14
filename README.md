# VR Based Hand Joint Angle Tracking
Northwestern ME 495 Winter 2025
Author: [Zhengyang Kris Weng](https://wengmister.github.io/)

## Overview
This project integrates VR and robotics to track hand joint angles in real time. It consists of two main components: a Unity-based VR application that captures joint angle data using Meta’s OpenXR SDK (previously using Oculus SDK) and broadcasts this data via UDP, and a C++ ROS2 node that receives the data, publishes it to a ROS2 topic, and provides a visualization UI. Additionally, the system can interface with a robotic hand for remote control.

## Schematic

![image](https://github.com/user-attachments/assets/3b029a6e-b938-49da-9050-409bfc95cc9b)

## Requirements

- Unity 6
- [Meta Interactino SDK](https://developers.meta.com/horizon/documentation/unity/unity-isdk-interaction-sdk-overview/)
- OXR Hand

## Development

There are two parts to this project - first part mostly focuses on developing unity app with C# and VR utilities to capture and compute joint angle data. The second part is mostly C++ ros2 project for receiving and data publishing.

The VR app was developed in Unity with meta SDK. Originally, I developed a version of this app using the legacy Oculus VR SDK since it's been released for some time and there are plenty of documentations of it.  

![quest-hand-tracking-0](https://github.com/user-attachments/assets/68ea856d-b601-449d-8e76-2763f2d7fafe)

However, I wasn't quite happy with the VR visuals, so I evantually switched to the new OpenXR SDK with Meta's building blocks. This allowed me to utilize some existing prefabs and features unique to meta Quest to develop an app with see-through background and synthetic hand overlay. There was much fewer documentation on OXR Hand usages, so navigating through this SDK was quite challenging. In addition to displaying joint angles in the VR Headset, I also made it broadcasting this message through UDP, to be received by a client PC through socket. This is where the C++ program will play its part and integrate with ROS2.    


![quest-hand-tracking-OXR](https://github.com/user-attachments/assets/7a6bdc8c-7d12-4ca9-99d1-74bc6c4448e3)

The C++ ros2 node, running on the client PC will listen to all incoming addresses on the defined port, and publishes received data to a ROS2 topic for downstream usages. It also creates a UI for visualizing the joint angles received.

## Quickstart

Deploy Unity Project via Unity Editor
- Select `Meta Quest 3/3s` as build target
- `Build and Run`

On ROS2 system:
- `git clone https://github.com/NU-MECH-ENG-495/project-wengmister-vr-hand-tracking.git`
- `cd project-wengmister-vr-hand-tracking`
- `rosdep install --from-paths src --ignore-src -r -y`
- `colcon build`
- `. install/setup.bash`
- `ros2 launch hand_tracking_quest hand.launch.xml`



Alternatively, to remote control [robotic hand](https://github.com/wengmister/Dex_Hand_MSR):
- `git clone https://github.com/wengmister/BiDexHand.git`
- `cd BiDexHand`
- `rosdep install --from-paths src --ignore-src -r -y`
- `colcon build`
- `. install/setup.bash`
- `ros2 launch hand_motion_shadowing shadowing.launch.xml cam:=quest`

## OOP Considerations

- **OOP Structure and Encapsulation:**
  - The node is implemented as a C++ class (`HandTrackerQuestNode`) that inherits from `rclcpp::Node`, encapsulating all functionality (UDP reception, message parsing, and publishing) within a single object.
  - The design separates public interfaces from private implementation details, making the code modular and maintainable.

- **Smart Pointers:**
  - Utilizes ROS2’s smart pointers, for instance, the publisher is created using a `std::shared_ptr` (`rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr`), ensuring automatic memory management.
  - The node itself is instantiated as a shared pointer in `main()`, promoting robust resource handling and avoiding manual memory deallocation.

- **Regular Expressions (Regex):**
  - Implements `std::regex` in the `parseJointAngles` method to extract floating-point numbers from the incoming UDP messages.
  - The regex pattern (`[-+]?[0-9]*\\.?[0-9]+`) is designed to capture signed numbers with optional decimals, ensuring flexibility in parsing different numeric formats.

- **Multithreading:**
  - Two separate threads are employed:
    - **Receiver Thread:** Continuously listens for UDP packets, processes incoming data, and pushes messages into a queue.
    - **Processing Thread:** Waits for messages in the queue, extracts joint angles using regex, and publishes the results.
  - Thread synchronization is handled using `std::mutex` and `std::condition_variable` to protect shared resources (i.e., the message queue) and coordinate thread activity.
  - Threads are properly managed with clean startup and graceful shutdown (via the `running_` flag and joining threads in the destructor).

- **Error Handling and Resource Management (RAII):**
  - Error scenarios (e.g., socket creation/binding failures) are logged using ROS2 logging mechanisms, providing immediate feedback during runtime.
  - The destructor ensures that resources like threads and the socket are cleanly released, following RAII principles to prevent resource leaks.

- **Additional OOP Considerations:**
  - Parameters (e.g., port number, debug flag) are declared and initialized through the node’s constructor, making the class configurable and extensible.
  - The code attempts to adjust thread priority using POSIX scheduling functions, reflecting a consideration for performance in time-sensitive applications.

## QT Gui
GUI:    
![image](https://github.com/user-attachments/assets/38c1c132-dfe4-4926-92d2-5c790d5c0728)


## Demo
VR Screencast:  

![quest-hand-tracking-OXR FinShort](https://github.com/user-attachments/assets/028d226d-eabc-4ab9-a0d7-5991a8760815)


Robotic hand remote control:    
![quest-hand-tracking-OXR Remote](https://github.com/user-attachments/assets/8e70620f-20d8-470f-9a47-385d7f58150f)

