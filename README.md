## Obstacle avoiding robot - RTOS
This project requires to design and build a robot system that can  run  straight  and  avoids  obstacles. A program based on Real-time Operating System (RTOS). The  complete  model  should  be  tested  in  the  location  as shown in the following picture.

![Objective](https://user-images.githubusercontent.com/100269450/186713958-d3e63f4b-10c2-4ffc-a3bc-4bb256d98516.jpg)

## Hardware
![Hardware](https://user-images.githubusercontent.com/100269450/186714269-275f9893-3c41-4e53-bc2d-23ecf62d29ea.jpg)

## Software Used :
* [Arduino IDE](https://www.arduino.cc/en/software)
* [Blynk website](https://blynk.cloud/dashboard/global)

## DC Motor Speed Control
Motor speed control using encoders. Encoder are connected to the hardware interrupt of the microcontroller(Nano). A timer using the timming interrupt service routine is used to measure the motor rotation speed. System Identification is performed to approximate the DC motors as 2nd order system. PID controller is implemented for speed control.<br/>

## High Level Control
* Control via ESP8266 blynk app : 
This functionality is added for the control of robot via ROS running your smartphone 
* Tutorial blynk :        [Blynk](https://github.com/blynkkk/blynk-library)

## Acknowledgments
* [Tuning a PID Controller with Genetic Algorithms](https://www.youtube.com/watch?v=S5C_z1nVaSg&t=2s)
* [PID Speed and PID position](http://arduino.vn/result/5401-pid-speed-position-control)

## Video Demo
* [DEMO](https://github.com/HLinhNguyen283/Obstacle_avoiding_robot-RTOS/blob/master/Images%26Demo/RTS_N33_Nhom3_Demo.mp4)


