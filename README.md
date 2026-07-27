# ECE346 – Intelligent Robotic Systems Labs

## Overview

The second branch of this repository (SP2026) contains the work completed with my group as part of the class **ECE346 – Intelligent Robotic Systems**, that I took in Princeton University during Spring 2026.

The course introduces the software components required to build autonomous robotic systems. Each laboratory focuses on a different aspect of robotics, from ROS programming and motion planning to collision avoidance and decision making, before bringing all these concepts together in a final project.

Rather than developing an application from scratch, I completed the programming tasks associated with each lab using the provided framework. My contributions include the implementation of the required algorithms, integration with the existing codebase and validation of the proposed solutions. In contrary, the final project gave an opportunity to decide on our own implementation to solve a robotics problem.

The class used the ROS2 environment to test new concepts in simulation and on a small-scale car. The car was built by the TAs especially for the class, and was controlled manually with a PS4. A large car track was also created to reproduce different paths and intersections.

Most of the semester was dedicated to understanding new concepts through labs, in simulation and with the car. After this, two weeks were dedicated to the final project, from ideation to presentation and demonstration.


<p align="center">
  <img src="RViz visualization tool.png" width="700" alt="Visualization tool">
  <br/>
  <em>Simulation of the car, represented in orange, on the tracks</em>
</p>

<p align="center">
  <img src="truck.jpeg" width="700" alt="Picture of the truck">
  <br/>
  <em>Small-scale car used during the semester</em>
</p>

<p align="center">
  <img src="tracks.jpeg" width="700" alt="Picture of the tracks">
  <br/>
  <em>Tracks created to run the real-life experiments</em>
</p>

---

## Course Objectives

Throughout the different laboratories, the course covers topics including:

- Robot Operating System (ROS2)
- Motion planning
- Trajectory optimization
- Collision avoidance
- Decision making under uncertainty
- Robot control
- Autonomous navigation
- Safe robotics principles

The final project combines these concepts into a complete autonomous robotics pipeline. The goal of this project was to build a safety filter for the car, letting the human control the car as wanted only if the proposed action is safe. 

---

## Contributions

With my group, I completed all required programming assignments, including:

- implementation of the algorithms requested in each laboratory
- integration of the implemented components into the provided software architecture
- debugging and validation in simulation
- implementation in real life with the provided small car and streets reproduction in the lab
- completion of the final project.

The original framework, simulation environment and laboratory instructions were provided as part of the course. The algorithmic implementations contained in the designated lab exercises and project tasks are my own work.
Everything can be found in the second branch of this repository (lab instructions, labs and project).

The topic and goals of each lab are presented below : 

| Lab | Topic | Accomplishments |
|---|---|---|
| Lab 0 | Introduction to ROS2 | Get familiar with basic ROS2 concepts (publishers, subscribers, ...), create a goal-reaching controller for the robot (that doesn't take into account the tracks) |
| Lab 1 | Trajectory planning with ILQR | Implement the ILQR algorithm for the dynamics of the car, deduce a feedback control policy from it, use ILQR to create a receding horizon planner |
| Lab 2 | Small-scale truck | Get the small-scale truck running, test it with the algorithms created in the previous two labs |
| Lab 3 | Collision avoidance | Include static obstacles in the ILQR algorithm to avoid them, use forward reachable sets to avoid a dynamic obstacle (another car in the simulation) |
| Lab 4 | MDP and POMDP | Implement an MDP for the negociation of a T-intersection, use value iteration and policy iteration to determine the actions to take, replace it with a POMDP to account for the uncertain position of the other car in the intersection |
| Lab 5 | Behavioral cloning | Collect data by manually running the car along the tracks, create a neural network that is trained offline on the collected data, test the policy deduced by the network on the car |

Finally, the goal of the final project was to create a safety filter for the car : if the action the human wants to take with the car is considered unsafe, the safety filter takes over and decides on a better action to take to avoid danger. 

Considering the time and ressources, we decided with the group to use a cost-based implementation for this problem. To decide if the proposed action is safe, we compute the cost of taking that action a few times more and then trying to recover with ILQR. Through different tests, we determined a threshold on that cost that indicated wether the action was safe or not. If the cost is higher than the threshold, the safety filter takes over. Otherwise, the action the human wants to take is safe enough and can go through. 

When the safety filter takes over, there are again two options. First ILQR finds the best action to be taken, without considering what the human wants to do. If the cost of taking that action and continuing with the safest trajectory is low enough, the safe action is the on that is sent to the car. But if the cost of the safe action and the safe trajectory is too high, the car has to brake immediately and come to a stop to avoid a collision or going off tracks.

---

## Repository Organization

All the files that were needed during the labs and the projects can be found in [ece346](https://github.com/ines-duranton/Robotics-project/tree/projectAndLabs/src/racecar_ece346/ece346).

The folder is organized according to the different lab sessions and the final project.

```text
lab0/
lab1/
lab2/
lab3/
...
final_project/
```

Each directory contains:

- instructions for each task
- starter code provided for the course
- implementation files completed during the lab
- launch and configuration files
- documentation and scripts when required

> **My work is primarily located in the implementation (`.cpp`, `.py`) files inside each laboratory folder and in the final project directory.**

---

## Learning output

This course emphasized both robotics algorithms and software engineering practices.

Beyond implementing individual algorithms, I learned how to:

- understand and extend an existing robotics framework;
- work with modular software architectures;
- integrate new functionalities into a larger codebase;
- validate algorithms through simulation;
- debug interactions between multiple software components.

---

## Acknowledgements

This repository is based on the teaching material provided for **ECE346 – Intelligent Robotic Systems**.

The framework and course infrastructure were provided by the teaching staff. My contribution consists of completing the required laboratory exercises and the final project implementations.
