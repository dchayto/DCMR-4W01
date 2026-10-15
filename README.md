# DCMR-4W01 - a custom mobile robot project
DCMR-4W01 (Diana Chaytor Mobile Robot - 4-Wheeled 01) is a personal project I've been working on, with the primary aims being to familiarize myself with the ROS2 stack, and to explore some basic controls (and in the future, autonomy) concepts.

To briefly summarize the status of the project: the robot is up and running to a basic degree - it drives well (aside from lateral movements, which it struggles with due to poor wheel traction), and is teleoperable using keyboard controls from an offboard computer. There are a lot of future developments I would like to explore (namely adding a camera feed to aid teleoperation, looking towards LIO SLAM, and afterwards some basic autonomy), but these will require (or at least benefit greatly from) a series of hardware improvements - mainly targeting improvements towards the electrical performance of the system and replacing some cheap COTS parts. At the moment, I am trying to decide between pursuing some of these developments, and shelving this project at its current stage for the time being in favour of working on a new robotics project.

## Table of contents
-> put TOC here

### Details of the current prototype build:
For ease-of-sourcing and prototyping, the platform was built using a variety of common COTS components, many of which I already had available. For this prototype, a Raspberry Pi 3B+ running Ubuntu server 22.04 was used as the compute module, and an Arduino Due was used as a motor control module - while not ideal for these purposes, these were selected out of convenience, as I already had them laying around. During untethered operation, the Due is powered off of the Pi via USB connection, and the Pi is powered off a universal power supply that I had from my FYDP.

The system is actuated by 4 FIT0458 motors, which are driven off 2 L298n motor drivers. The motors have built-in encoders for wheel odometry measurements. Also in the current sensor stack is a BNO085 9-DOF IMU. For onboard auxiliary power, 4 rechargable AA batteries were run in series. This power circuit is switched by a 5V relay module controlled by the Due's power lines (i.e., when the Due switches on, the auxiliary power also comes online).

Completing the electrical design is a small breadboard circuit, which consists of a main power bus with some voltage stabilizing capacitors, voltage dividers to bring encoder ticks to levels compatible with the Due's GPIO pins (3v3), and some routing for various connections. 

The robot also has 4 ultrasonic sensors integrated into its chassis for future collision detection/last-measure obstacle avoidance - though they haven't been integrated yet due to an already-present cluttering issue. All new components were purchased off DigiKey or Amazon.

### Current status of the project
The system is able to run untethered (controlled via teleoperation over wifi). At the moment, this is being done by SSHing into the Pi from my dev computer, but with some amount of setup this could be configured to run over ROS through my LAN - my dev computer is running a newer ros2 version than the pi though (Jazzy on dev computer, Humble on Pi - running outdated Ubuntu, so had to run outdated ros2 as well), so there may be some compatibility issues (which I had no desire to work out for the time being).

Wheel PIDs were written into the motor control module firmware, and seem to be performing well (at least when battery power remains high - as the power drains, the wheel speed PIDs seem to have trouble tracking inputs, requiring higher command inputs to achieve a given speed).

IMU and encoder measurements were integrated into the sensing stack, and are currently being used for closed-loop control on both the wheel speed controllers, and the body velocity controller. Other than issues resulting from excessive wheel slip (the COTS mecanum wheels I sourced have very poor traction on most surfaces), the robot is performing reasonably well - noting trade-offs in controller tuning that I opted for to try to mitigate the wheel slip issue.

### Future plans
The long-term aim of this project was always to touch on some higher-level autonomy concepts, but given that this needed to be built towards, it was always going to be fairly low on the priority list (many basic requirements needed to be met first). While it would be nice to continue with a robot I designed and built myself, it might be best to look at some of these concepts with a more vetted platform (though any capable of handling these new requirements are likely to be very expensive, and at lower price points could carry many of the issues left to resolve with this project - if not more).

Before starting on some of the projections listed below, there is some "cleanup" of the prototype that needs to be done - see the "housekeeping items" section below. 

#### mapping and localization (LIO SLAM)
- look at slam_toolbox and Nav2. would require integrating a lidar into project (space already allocated for it). likely would not be able to run onboard due to limited compute power of the Pi (CPU cores are already running at ~70% capacity under load with the current running processes), but may be able to forward lidar data and do offboard processing - i.e., handle high-level localization, mapping, and planning on desktop PC, and just run lower-level controls onboard (velocity control, obstacle avoidance, dead reckoning, etc.)
- depending on the feasibility of offboarding SLAM and mapping (e.g., considering how realistic the data pipelining is, and what sort of update latency would i be looking at), it may make more sense to just opt to purchase a better compute module - a Raspberry Pi 4B+ should be a drop-in upgrade for the features i actually care about (mounting and footprint compatibility), and would provide a far more performant processor, and also allow me to select an option with more built-in memory. The upgraded computer would also certainly be able to run Ubuntu server 24.04, which would allow me to match ros2 versions with my dev PC. It also supports 5GHz wifi, allowing for lower network latency.

#### position control
not yet added due to lack of global reference frame at the moment. also, position estimate is likely to be poor given current sensor stack and traction issues (can rely on wheel odometry reasonably well for forwards and turning motions, but any derived lateral motion estimates are completely untrustworthy).

#### planning and autonomy
- once mapping, localization, and position control have been realized, i can look into running some basic autonomous missions (given the platform, this will be limited to "go from location A to B and inspect the area").


### Housekeeping work
#### electrical cleanup: 
custom PCB -> the breadboard circuit, power bus, and several of the COTS components would be much better replaced with a custom PCB (best designed as a hat board for the Due). the L928n motor drivers used for simplicity are pretty inefficient (50-70%), and should be replaced with either a more performance COTS driver, or, preferably, integrated into the custom PCB
-> currently running power bus off 4 rechargable AA batteries (2300mAh) - need to look for some higher-capacity batteries

#### COTS component swap-out
**wheels**
used very cheap mecanum wheels sourced off amazon for this project. rollers are made of a hard, slippery plastic (assumed to be ABS or HDPE), and have very poor traction. this leads to excessive slipping in the roller-axis directions, which makes holonomic movement (which was supposed to be the advantage of using the mecanum wheels) inconsistent and inefficient, in addition to having negative implications on the validity of the encoder odometry. the wheels have especially poor traction on the vinyl flooring in my apartment (they drive a little better on a rubber mat or on thin rugs), and they tend to get caught up in the carpeted flooring elsewhere in the apartment.
I've tried my best to smooth out the controls - i've increased the integral and derivative gains in the driving direction, and weighted the linear movement readings from the IMU higher than i might otherwise have in order to try to stabilize about the x=0 operating point - so that it can at least drive (albeit slowly) in this direction, but i fear i am approaching the limit of what can practicably be done with this hardware (both in terms of sensing and state estimation with the current compute and sensor stack, and on compensating for wheel slippage).
These wheels should be swapped out with either more performant mecanum wheels (i.e., ones with better roller traction), or with regular wheels with better traction (this would require disabling lateral movement commands).

**L298n motor drivers:**
as stated in the electrical section, the motor drivers used for this stage of the prototype were selected for simplicity, not performance - they are horribly bulky and inefficient, and the headers on the board chosen require the use of DuPont connectors, which are mechanically unreliable, and generally perform poorer than e.g., JST PH connectors. As mentioned, they should be swapped out with a more performance COTS driver, or preferably, integrated into a custom PCB.

of course, when these changes are implemented, wheel and body velocity controllers may require some adjustments to match the electrical changes (if motor supply voltage, or available current changes)

software cleanup:
write URDF -> get some experience working with ros2_control as well as simulation tools (Gazebo/RViz2/MoveIt). seemed like overkill for this project, but seems very useful for more complicated projects with non-static joint transforms (e.g., if i wanted to work on an arm next)
input handler rewrites -> want something more reliable than the little TTY interface i wrote myself. also want something that can handle combined inputs that weren't pre-coded
-> integrating ultrasonic sensor readings into project (hoping to use for collision detection/obstacle avoidance)

