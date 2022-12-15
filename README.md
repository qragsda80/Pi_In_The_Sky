# Pi_In_The_Sky

## Planning

### The Project

Fly a drone controlled by a remote and be able to collect data on an altimeter, and then be able to land the drone.

### Success Criteria

Criteria to determine if our project is a success:

1) Drone is able to take off and fly
2) Drone is able to be controlled by a remote
3) Drone can safely land
4) Data can be safely extracted and read

### Available Materials

Most of the materials we would need for our project we have in our lab. Any additional materials we need can be bought from Amazon. Currently, the only planned purchase is the drone motors. 

### Materials List

1) ABS 3D print material for frame
2) Ninjaflex 3D print material for propellor guards
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

Here's some links on PID to learn:

[PID Link 1](https://blog.devgenius.io/flying-a-drone-with-python-pid-control-7001a41f54ac)

[PID Link 2](https://www.technik-consulting.eu/en/optimizing/drone_PID-optimizing.html)

[PID Link 3](https://www.mathworks.com/videos/understanding-pid-control-part-1-what-is-pid-control--1527089264373.html)

### Build

#### Design

We are trying to constrain the size and mass of our design as much as possible to ensure maximum flight smoothness. We are referencing the [DJI Tello](https://m.dji.com/product/tello) for our rough dimensions. 

#### Special Choices

1) We are using Tello Drone specific motors for easy specification accesebility. 
2) During testing, we will make the frame out of acrylic to be conscious of material used in 3D printing (we will need to make many iterations and don't want to waste material)
3) In our final design, we will utilize ABS material for our frame, but use Ninjaflex for 

### Risk Mitigation

| Risk  | Possibility | Mitigation |
| ------------- | ------------- | ------------ |
| Drone Loses Control  | HIGH | Add a Killswitch |
| Overheating | HIGH | Wear gloves in post flight recovery |
| Frame cracks | HIGH | Build detachable propellor guards |
| Drone activation unknown | Medium | Add ON light(green) |
| Drone Explodes  | Low  | Have fire extinguisher on Standby |


| Testing Risks | Possibility | Mitigation |
| ------------- | ------------- | ------------ |
| Doesn't respond w/ controller | HIGH | Build controlled testing piece (string) |
| Frame reconstruction | Medium | Laser cut body & 3D print motor part |


### Proposed Schedule & Milestones

| Week  | Tasks | Milestone |
| ------------- | ------------- | ------------ |
| 12/12 - 12/15 | General - Work On Project Proposal | Finish Proposal |
| 1/3 - 1/6 | Build Drone in CAD / PID Stablization Code | No Milestone / Have stabilizing motors |
| 1/9 - 1/13 | Continue CAD Drone / Troubleshoot PID & Work on Joystick Control | Print 1st prototype drone / Finalize PID Stability |
| 1/16 - 1/20 | Assemble and Test 1st prototype | Complete Prototype |
| **Quarter Ends (1/20)** |TRY TO HAVE A PROTOYPE IN TESTING | Enjoy the weekend |
| 1/23 - 1/27  | Reiterate design / Debug & Improve code  | No Milestone |
| 1/30 - 2/3 | Assemble and Test 2nd Prototype / Test Joystick Code | 


### Optimizations



