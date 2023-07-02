# Robot_Vehicle

## Video
<a href="https://www.youtube.com/watch?v=AxMXdjCPqoc" target="_blank">![Watch the video](https://github.com/ArtemHW/images/blob/main/robot_vehicle_video_screen.png)</a>
## Description
The purpose of this project was to develop a prototype of a Bluetooth-controlled robot machine. However, the machine incorporates certain limitations that take precedence over the commands received via Bluetooth, ensuring the safety of operation. These limitations primarily involve two aspects. Firstly, the machine's movement is constrained within a specific area, demarcated by a black bar on the ground. Secondly, the machine must navigate around various obstacles, such as walls or living creatures.

To adhere to these restrictions, the robot car is programmed to react accordingly. If it detects an obstacle or approaches the boundaries of the designated area, it will automatically move in reverse to avoid collisions or exceeding its permitted range of movement.

The microcontroller stm32f401RE is utilized for control in this project. To ensure smooth driving, a PID control algorithm has been implemented. The movement of the car is powered by two DC motors, and their speed is measured by optical encoders, which are connected to the microcontroller. It is important to note that the motors are not directly powered by the microcontroller; instead, they receive power from a 9V battery.

The microcontroller regulates the motors through a combination of Pulse Width Modulation (PWM) signals and a DC motor driver. Additionally, several modules are connected to the microcontroller. These include a bluetooth module HC-05, an ultrasonic module HC-SR04, a line following module, and an accelerometer/gyroscope module (MMA8452X). The accelerometer/gyroscope communicates with the microcontroller via the I2C bus, and you can find the library for this module in the "My_API_for_modules" repository.

The purpose of incorporating the accelerometer/gyroscope module is to determine the inclination angles of the robot and precisely identify moments of acceleration or deceleration.

## Block scheme
![App Screenshot](https://github.com/ArtemHW/images/blob/main/Robot_Vehicle_block.drawio.svg)
## PID controller scheme
![App Screenshot](https://github.com/ArtemHW/images/blob/main/PID.png)