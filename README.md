# Pi_In_The_Sky

## The Project

Times are changing. Engineers such as Quinn & Shrey are super chill, but need to stay current in order to keep being chill. Drones are the next big thing in engineering, and building one successfully: that's super chill. As engineer's in the modern times, our job is to help the world, and be chill while doing it, because we want to set a good example for others who think Engineering is for not-chill people.

What to we do? **Fly a drone controlled by a remote and be able to collect data on an altimeter, and then be able to land the drone.** The very concept of the drone screams modern while including glimpses of chillness. Collecting data while in the air adds anotfher aspect of chillness to the build. Although the drone in it's current proposal only collects flight data, it lays the foundation for additional sensors and functionalities never dreamed of before in a High School Engineering class. This drone, through it's simplicity, lays a foundation of chillness in Engineering for generations to come.

## Planning

<details>
<summary>Check out our Project Planning!</summary>

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
6) Other small props (undecided)

### Code

The most difficult challenge to overcome in code is to be able to program the joysticks on a controller to accurately control the drone

#### Motor Control

The diagram below shows how the motors will need to operate in order to move the drone is specific ways

![download](Images/download.png)

#### Code Technicalities

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

We have prototyped 2 rough drone designs in OnShape

First design Idea: 

![firstdronedesign](Images/firstdronedesign.JPG)
<img src="https://github.com/qragsda80/Pi_In_The_Sky/blob/main/Images/firstdronedesign.png?raw=true" alt="First Drone Design" width="500">

Second Design Idea:

![secondronedesign](Images/seconddronedesign.jpg)

#### Special Choices

1) We are using Tello Drone specific motors for easy specification accesebility. 
2) During testing, we will make the frame out of acrylic to be conscious of material used in 3D printing (we will need to make many iterations and don't want to waste material)
3) In our final design, we will utilize ABS material for our frame, but use Ninjaflex for our propellor guards and landing buffer. This allows us the have slightly more flexibility and durability. If the drone crashes, the ninjaflex could prevent it from cracking completely.
4) Our propellor gaurds will be made detachable to the main frame (snap fit). If the drone crashes, the chances are more likely it will land on the propellor guards than any other part of the drone, so making the guards detachable allow us to only need to reprint a piece rather than the entire frame.

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
| 12/18 - 1/2 | Winter Break | Chill |
| 1/3 - 1/6 | Build Drone in CAD / PID Stablization Code | No Milestone / Have stabilizing motors |
| 1/9 - 1/13 | Continue CAD Drone / Troubleshoot PID & Work on Joystick Control | Print 1st prototype drone / Finalize PID Stability |
| 1/16 - 1/20 | Assemble and Test 1st prototype[^1] | Complete Prototype |
| **1/20** | **Quarter Ends** | **Try to have a Prototype in Testing** |
| 1/23 - 1/27  | Reiterate design / Debug & Improve code  | No Milestone |
| 1/30 - 2/3 | Assemble and Test 2nd Prototype[^2] / Test Joystick Code | No Milestone |
| 2/6 - 2/10 | Continue iterating Design & Code | Complete 2nd Prototype if not already done |
| 2/13 - 2/28 | Work towards building Final Prototype[^3] | Finish Final Prototype (2/28) |
| 3/6 - 3/31 | Design Refinement & Final Tweaks | Ready to launch |
| **3/30** | **Quarter Ends**| **Ready to Launch** |
| 4/3 - 4/7 | First Project Launch | Launch Drone and Collect Data |
| 4/10 - 4/14 | Modify Build/Code & Prepare to relaunch next week | Analyze 1st launch data & have drone ready for relaunch |
| 4/17 - 4/21 | Second Project Launch | Launch Drone & Collect/Analyze Data |
| 4/24 - 4/28 | Buffer Week | Catch up on schedule |
| 5/1 - 5/5 | Final Project Documentation | Add Analyzed Data & Finish Final Project Documentation |
| 5/8 - 5/19 | Buffer Weeks | Catch up on Schedule |
| **5/19** | **Project Due** | **Submit GitHub Repo on Canvas** |
| 5/22 - 6/9 | EMERGENCY PROJECT WORK (if required) | FINISH PROJECT / Chill & Enjoy Flying Drone |
| **6/9** | **LAST DAY OF SCHOOL** | **PLACE DRONE ON WALL OF FAME** |



[^1]: First Prototype is to be a controlled flight (attached to string and not actually free flying)
[^2]: Second Prototype is to be a slightly free flighted prototype (will not be flying above 5 feet)
[^3]: Final Prototype is a fully functional drone prototype very similiar to the final product

 </details>

