# Project Proposal: C++ Hand Joint Angle Tracking on Meta Quest 3s
Author: Zhengyang Kris Weng
Northwestern University ME495 Winter 2025

## Overview
This project aims to develop a **C++ application** that harnesses the **Meta Quest 3s**’s hand-tracking features to measure, visualize, and log real-time hand joint angles.

![image](https://github.com/user-attachments/assets/a6680d59-4a9e-4aee-8b4a-3df9e20ea0ac)

## Background
I'm currently developing [a biomimetic robotic hand for my winter project](https://github.com/wengmister/Dex_Hand_MSR). I've achieved some basic motion shadowing using usb camera input. VR Headsets, with their depth camera and native hand gesture recognition, could be a great alternative input modality to data collection for my robotic hand. Therefore, I aim to design and develop a program that facilitates data collection and transfer from the VR headset.

---

## Objectives
1. **Integrate Hand-Tracking SDK**  
   Utilize Quest 3’s built-in cameras and SDK to capture hand and finger joint data.
   - Develop a program using unity to capture hand angle
   - Use the same unity program to transfer data via UDP or TCP/IP

2. **Compute Joint Angles**  
   Convert 3D positions or quaternions into meaningful angles for each finger segment.
   - Utilize the XRHand/OVRHand class to acquire joint angle data

3. **Data Transfer and Logging**  
   Store angle data for offline analysis or debugging (CSV/JSON).
   - Using websocket, capture the joint data locally
   - Save streamed joint angle data locally in CSV or JSON

4. **Project Integration**
   - Using ROS2 pub/sub, publish joint angles to the robotic hand
   - Support multiple camera input modality

5. (Stretch) **Gesture Recognition**  
   Design a robust C++ codebase for potential expansions

6. (Stretch) **Real-Time Visualization**  
   Display angles in the VR display that matches joint angle read from the headset cameras

---

## Resources
- **Hardware**: Meta Quest 3s, Windows development PC (for unity), and Linux development PC (for ROS2)
- **Software**: C++, Unity Editor, C# (for unity), ROS2 

---
