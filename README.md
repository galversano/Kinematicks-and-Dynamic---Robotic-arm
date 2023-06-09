# Kinematicks-and-Dynamic-Robotic-arm
Matlab program for Robotic arm (DOF RPPRRR)
R-rotation
P-prismatic

# Description of the model robot
The robot will include 6 DOF.

Rotation Prismatic Prismatic Rotation Rotation Rotation
**(theta1 , l1 ,l2 , theta2, theta3, theta4)**

Here you can see a video for stick model in ROS2-Rviz:



https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/46b216fb-d5f7-47be-b348-594a68f571b1



# Forward Kinematicks
Calculate the Forward Kinematicks for robotic arm DOF (RPPRRR)

    point1=[0, 0, pi/2, 0, 250,0];
    point2=[0, pi/2, pi/2, 0, 250, 250];
    point3=[pi, 0, 0, 0, 500, 500];
    point4=[pi, pi/2, pi/2, pi, 500, 500];
    point5=[pi, 1.5*pi, 0, pi, 0, 0];

![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/2cd94530-bdd9-4cee-96d3-244e071bf2a0)![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/793c650c-1c64-46a1-b082-d7398d3df567)

![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/c2f29975-0c4d-4303-aad0-25edc0f66b10)![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/3f8a355b-3e9c-42d0-ac5e-57615373feea)
![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/6018ec23-0c18-4ee2-b026-51c3b0f2113f)


You can change the point(1,2,3,4,5) and get diffrent result

# Discritization
All the points the robot can reach. 

![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/e52abb79-4db8-4f20-9797-992a3ef12849)
(takes long time to run)


# Inverse Kinematicks
Get input for X,Y,Z and orientation matrix.
the result will be the value of each DOF.


# Jacobian 
Calculate the Jacobian (parametric) 

  ```syms theta1 theta2 theta3 theta4 ; ```
  
  
  ***Result:***
 ``` 
J= [  150*cos(theta2)*sin(theta1)*sin(theta3) - 150*cos(theta1)*cos(theta3) - l2*cos(theta1) - 151*cos(theta1), 0, -sin(theta1), 150*cos(theta1)*sin(theta2)*sin(theta3),   150*sin(theta1)*sin(theta3) - 150*cos(theta1)*cos(theta2)*cos(theta3),                        0]
[- 151*sin(theta1) - 150*cos(theta3)*sin(theta1) - l2*sin(theta1) - 150*cos(theta1)*cos(theta2)*sin(theta3), 0,  cos(theta1), 150*sin(theta1)*sin(theta2)*sin(theta3), - 150*cos(theta1)*sin(theta3) - 150*cos(theta2)*cos(theta3)*sin(theta1),                        0]
[                                                                                                         0, 1,            0,             150*cos(theta2)*sin(theta3),                                             150*cos(theta3)*sin(theta2),                        0]
[                                                                                                         0, 0,            0,                                       0,                                                -cos(theta1)*sin(theta2), -cos(theta1)*sin(theta2)]
[                                                                                                         0, 0,            0,                                       0,                                                -sin(theta1)*sin(theta2), -sin(theta1)*sin(theta2)]
[                                                                                                         1, 0,            0,                                       1,                                                             cos(theta2),              cos(theta2)]
```

# Statics
An analytical calculation of the moments and forces required in the robot's joints to lift a mass of M at the end effector.

  ***Result:***

```tau= transpose([0 Mg 0 150*Mg*cos(theta2)*sin(theta3) 150*Mg*cos(theta3)*sin(theta2) 0 ])```

# Motion planning 
For calculate the Motion planning, the inputs: start point and end point (X,Y,Z) and oreination (Roll,Pitch,Yaw)

Motion planning will base on acceleration. const speed and deceleration (end effector):
![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/ba3e0d26-7033-418b-ac90-a866fb72df6b)

The position for the end effector (each dt) will look like:
(point [0 400 1200] -> [0 300 800])
![image](https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/8e8a7fa6-7aa4-4dfe-8540-49f42ebc9566)


# Simulation
The simulation will include a stick model for the robotic arm.
Will look like this:




https://github.com/galversano/Kinematicks-and-Dynamic---Robotic-arm/assets/66177443/d1a48807-78cf-4abb-91af-57cfff508698






