# Pi_In_The_Sky

## Planning

### The Project: 

Fly a drone controlled by a remote and be able to collect data on an altimeter, and then be able to land the drone.

### Success Criteria:

Criteria to determine if our project is a success:

1) Drone is able to take off and fly
2) Drone is able to be controlled by a remote
3) Drone can safely land
4) Data can be safely extracted and read

### Materials List

1) ABS 3D print material for frame
2) Rasberry Pi Pico
3) Altimeter
3) 500 mAh LiPo Battery
4) 4 motors
5) Custom Circuit board
6)
7) Other small props (undecided)

### Code

The most difficult challenge to overcome in code is to be able to program the joysticks on a controller to accurately control the drone

Motor Control: The diagram below shows how the motors will need to operate in order to move the drone is specific ways

![download](Images/download.png)

Coding:

I found this code snippet from online that moves a drone with joysticks using arduino:

``` c++

 LR = analogRead(A0);     // read analog joystick voltage from pin A0, returns an integer from 0-1023
 UD = analogRead(A1);     // read analog joystick voltage from pin A1, returns an integer from 0-1023
 LR = 5*LR/1023;          // convert to a voltage from 0-5V
 UD = 5*UD/1023;          // convert to a voltage from 0-5V


// convert analog voltages to motor speeds
 motor1speed = defaultSpeed+speedChange*(a*UD-a*LR);
 motor2speed = defaultSpeed+speedChange*(a*UD+a*LR-b);
 motor3speed = defaultSpeed+speedChange*(-a*UD+a*LR);
 motor4speed = defaultSpeed+speedChange*(-a*UD-a*LR+b);
  
```

### Build

### Risk Mitigation

### Optimizations



