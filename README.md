# arduinodronegyro
Sample gyro balancing codes for a homemade drone which I have made using Arduino. PID control is included in the piddrone files and manual control based on control statements is used in the older code files.

The circuit involves the use of 4 MOSFETs for each of the motors, all of which are connected to a LiPo battery. A seperate smaller battery is used to power an Arduino nano microcontroller connected to an MPU6050 gyro sensor to adjust the speeds of the motors. This can be seen more clearly in the schematic below: 

![alt text](https://github.com/ahmadkaleem123/arduinodronegyro/blob/master/Drone%20circuit.jpg)
