# Multi-Agent-System

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
Given a trajectory (the handwritten letter "a") in Jacobian space, the joint space trajectory can be obtained by solving the inverse kinematics of the robotic arm. For each point on the trajectory, the inverse kinematics is solved to obtain the joint angles, and then the end-effector coordinates are obtained based on the geometric relationships.

```
execute TwoLinkArm_TrajectorySolver.m
```

<p align="center">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/robot_arm.png" alt="image" width="30%" height="auto">
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/inverse_kinematics.gif" alt="gif" width="30%" height="auto">
</p>


## **2.dynamic model control of a robotic arm**
The Lagrangian method is an approach used to derive the dynamics equations of robotic manipulators, which can be subsequently utilized for motion control and trajectory tracking. Given a trajectory (the handwritten letter "a") in Jacobian space:

```
execute TwoLinkArm_DynamicsModel.m
```

<p align="center">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/dynamic0.png" alt="image" width="20%" height="auto">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/dynamic1.png" alt="image" width="20%" height="auto">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/dynamic2.png" alt="image" width="20%" height="auto">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/dynamic3.png" alt="image" width="20%" height="auto">
</p>

## **3.stabilization and tracking control of a differential mobile robot**
<p align="justify">
In this part, I used a target-to-target PID controller to control the differential wheeled robot. In terms of specific implementation, I projected the reference trajectory onto the x-axis, y-axis, and orientation angle, obtaining three sets of position and orientation functions with respect to time. I then designed three PID controllers to track the aforementioned three trajectories, ultimately fusing them into the robot's linear and angular velocities, and further calculating the left and right wheel speeds.</p>





```
roslaunch diff_drive demo.launch
```


## **4.multi-agent formation**
<p align="justify">
In this part, I used the leader-follower method for the formation control of three agents in Gazebo, with the formation shape chosen as a triangular formation. The simulation environment sends the localization data of Robot1, Robot2, and Robot3, as well as the data from the laser rangefinders mounted on the robots, to their respective nodes. Run after compiling the ros workspace.</p>

```
roslaunch formation playground_gazebo.launch
roslaunch formation triangle.launch
```

Robot1 serves as the leader in the formation, sending its laser data to the leader node. The leader combines the laser data to generate motion and obstacle avoidance commands, which are then sent to the robot's base for control. The follower nodes receive not only their own laser data and position information but also the position information of the leader. They then calculate the control commands for the followers based on the differences in position (distance and yaw angle) between the leader and followers. By using a control algorithm, the followers gradually approach the desired position, thus achieving the formation control effect. 


## **5.multi-agent encirclement**
<p align="justify">
In the encirclement phase, the encircling robots approach the target in an arc-shaped formation, with several robots evenly distributed along a section of a circular arc, ensuring that the encircling robots are at equal distances from target. This approach is applicable to targets with different types of trajectories.</p>

<p align="center">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/tracking.png" alt="image" width="30%" height="auto">
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/catching.png" alt="image" width="30%" height="auto">
</p>
<p align="justify">
I uesd the Artificial Potential Field (APF) method for multi-agent encirclement tasks. The basic principle of APF is to model the multi-agent system as a set of particles subjected to virtual forces generated by attractive and repulsive potential fields. </p>

<p align="center">
    <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/simulate.png" alt="image" width="66%" height="auto">
</p>

<p align="justify">
During the encirclement process, each agent calculates the total force acting on it by summing up the attractive and repulsive forces resulting from the potential fields. The agents then update their positions and velocities according to these forces, allowing them to move towards the target while maintaining the desired formation and avoiding collisions.</p>

Since the code relies on specific hardware and cannot be directly reproduced, I have included a GIF here to show the implementation result.
<p align="center">
  <img src="https://github.com/MRHan-426/Multi-Agent-System/blob/main/.assets/encirclement.gif" alt="gif" >
</p>

