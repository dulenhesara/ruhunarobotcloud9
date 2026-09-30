🤖 Robot Challenge – Autonomous Competition Robot
📌 Project Overview

This project is an autonomous robot developed for the Ruhuna University Robot Challenge Competition.

The main goal of this project is to build a robot that can complete several challenges without direct manual control.

The robot uses sensors, motors, encoders, and control algorithms to understand its environment and perform different tasks.

🏆 Competition

Competition: Ruhuna University Robot Challenge
Robot Type: Autonomous Mobile Robot

The competition contains several stages. Each stage requires the robot to perform a different task.

Competition Stages

The main tasks included in the competition are:

🌀 Unknown Maze Solving
📦 Box Detection and Collection
⚽ Ball Collection
🧱 Wall Following
🎯 Ball Shooting

The robot must use sensors and programmed algorithms to complete these tasks.

🌀 1. Unknown Maze Solving

In the first stage, the robot enters an unknown maze.

The robot does not have a pre-programmed map of the maze.

It must:

Detect walls using sensors
Identify available paths
Decide which direction to move
Detect intersections
Detect dead ends
Navigate through the maze
Reach the required destination
Main Technologies
Distance sensors
Wheel encoders
Motor control
Maze-solving algorithm
PID control

The robot uses sensor information to make decisions while moving through the maze.

📦 2. Box Detection and Collection

In this stage, the robot needs to find and collect a box/object.

The robot must:

Detect the box
Move toward the box
Position itself correctly
Stop at the correct distance
Activate the robot arm
Grab the box
Lift and carry the box
Robot Mechanism

The robot uses:

Servo motors
Robotic arm
Gripper
Distance sensors
Limit switches

The gripper is used to hold the box securely.

⚽ 3. Ball Collection

In this stage, the robot needs to collect balls from the competition area.

The robot must:

Detect the ball
Move toward the ball
Position itself correctly
Collect the ball
Store or carry the ball

Sensors are used to detect the position of the ball and obstacles around the robot.

🧱 4. Wall Following

The robot must move while maintaining a suitable distance from a wall.

The robot continuously measures the wall distance using distance sensors.

For example:

        WALL
────────────────────────
        ↑
        │ Distance
        │
      🤖 ROBOT

If the robot gets too close to the wall, it moves away.

If the robot moves too far from the wall, it moves closer.

A PID controller can be used to maintain a stable distance.

PID Control

The basic idea is:

Target Distance
       ↓
Distance Sensor
       ↓
   Calculate Error
       ↓
   PID Controller
       ↓
Motor Speed Adjustment
       ↓
     Robot
🎯 5. Ball Shooting

In the final stage, the robot needs to shoot the collected ball toward the required target.

The robot must:

Position itself correctly
Detect or determine the target position
Aim the shooting mechanism
Activate the shooter
Shoot the ball toward the target

The shooting mechanism can use motors or other actuators depending on the robot design.

🔧 Hardware

The robot can contain the following hardware:

Main Controller
ESP32
Motors
DC gear motors
Encoder motors
Sensors
ToF distance sensors
IR sensors
Encoders
Limit switches
Other sensors required for object detection
Actuators
Servo motors
DC motors
Robotic arm
Gripper
Ball shooting mechanism
💻 Software

The robot software is developed mainly using:

Arduino / ESP32
C/C++
PID control
Encoder feedback
Sensor processing
Maze-solving algorithms

The software is divided into different modules so that each robot function can be tested separately.

🧠 Main Algorithms
1. Line / Motion Control

The robot controls the left and right motors according to sensor feedback.

2. PID Control

PID is used to improve the robot's movement and maintain accurate control.

It can be used for:

Wall following
Straight-line movement
Sensor alignment
Motor speed control
3. Encoder Feedback

Wheel encoders measure how much each wheel has rotated.

This allows the robot to estimate:

Distance traveled
Wheel speed
Robot movement
Difference between left and right wheel speeds
4. Maze Solving

The robot uses sensor readings to determine:

Wall present
Open path
Left path
Right path
Dead end
Intersection

The maze algorithm then selects the next movement.
