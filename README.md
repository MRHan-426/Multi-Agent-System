# Multi-Agent-System

<p align="center">
  <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/encirclement.gif" alt="gif" >
</p>

![status](https://img.shields.io/badge/status-archived-EB1923)
![last modified](https://img.shields.io/badge/last%20modified-04%2F28%2F2023-EB1923)

This repository is used to archive the code of the course **Intelligent Multi-Agent System**, presented by Prof Zhuping Wang. 

The laboratory part of the course includes the following topics: 
+ inverse kinematics simulation of a robotic arm
+ dynamic model control of a robotic arm
+ stabilization and tracking control of a differential mobile robot
+ multi-agent formation 
+ multi-agent encirclement

The course employs [Qbot2e](https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/QBot2e_Datasheet.pdf) robot for the experiments.


## **1.inverse kinematics simulation of a robotic arm**
Given a trajectory (the handwritten letter "a") in Jacobian space, the joint space trajectory can be obtained by solving the inverse kinematics of the robotic arm.
For each point on the trajectory, the inverse kinematics is solved to obtain the joint angles, and then the end-effector coordinates are obtained based on the geometric relationships.



<p align="center">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/robot_arm.png" alt="image" width="30%" height="auto">
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/inverse_kinematics.gif" alt="gif" width="30%" height="auto">
</p>


## **2.dynamic model control of a robotic arm**
## **3.stabilization and tracking control of a differential mobile robot**
## **4.multi-agent formation**
## **5.multi-agent encirclement**

## 2.Examples
Since the code relies on specific hardware and cannot be directly reproduced, I have included two GIFs here to show the implementation results.

+ **Example1:** \
Voice-activated mobile robot navigation. The intelligent car navigates to its destination, recognizing characters that may appear on plastic boards along the way, determining whether they have long hair and whether they are wearing glasses. Finally, it enters the parking area and outputs the recognition results.

<p align="center">
  <img src="https://github.com/MRHan-426/Chinese-National-College-SmartCar-Compeition/blob/master/.assets/example1.gif" alt="gif" width="66%" height="auto">
</p>


+ **Example2:** \
A collaborative work between a mobile robot and a robotic arm. The mobile robot navigates, recognizes images, and communicates with the robotic arm. The robotic arm then picks up the corresponding object type and places it on the mobile robot, which subsequently navigates to its destination.

<p align="center">
  <img src="https://github.com/MRHan-426/Chinese-National-College-SmartCar-Compeition/blob/master/.assets/example2.gif" alt="gif" width="66%" height="auto">
</p>

