# DCMR-4W01 - a custom mobile robot project
DCMR-4W01 (Diana Chaytor Mobile Robot - 4-Wheeled 01) is a personal project I've been working on, the primary aims of which were to familiarize myself with the ROS2 stack, to get some C++ coding experience, and to explore some basic controls (and in the future, autonomy) concepts.

To briefly summarize the status of the project: the robot is up and running to a basic degree - it drives reasonably well (aside from lateral movements, which it struggles with due to poor wheel traction), and is teleoperable using keyboard controls from an offboard computer. There are a number of future developments I would like to explore (namely adding a camera feed to aid teleoperation, looking towards LIO SLAM, and afterwards some basic autonomy), but these will require (or at least benefit greatly from) a series of hardware improvements - mainly targeting improvements towards the electrical performance of the system and replacing some cheap COTS parts. At the moment, I am trying to decide between pursuing some of these developments, and shelving this project at its current stage for the time being in favour of working on a new robotics project.
<br><br>

## Table of contents
[Prototype build](#prototype-build)
[Project status](#project-status)
[Future plans](#future-plans)
  [Mapping and localization (LIO SLAM)](#mapping-and-localization-lio-slam)
  [Position control](#position-control)
  [Planning and autonomy](#planning-and-autonomy)
[Housekeeping work](#housekeeping-work)
  [Electrical cleanup](#electrical-cleanup)
  [COTS component swap-out](#cots-component-swap-out)
  [Software cleanup](#software-cleanup)
<br><br>

### Prototype build:
For ease-of-sourcing and prototyping, the platform was built using a variety of common COTS components, many of which I already had available. For this prototype, a Raspberry Pi 3B+ running Ubuntu server 22.04 was used as the compute module, and an Arduino Due was used as a motor control module - while not ideal for these purposes, these were selected out of convenience, as I already had them laying around. During untethered operation, the Due is powered off of the Pi via USB connection, and the Pi is powered off a universal power supply that I had from my FYDP.

The robot's chassis is custom designed, and 3D printed from PETG. COTS mecanum wheels were sourced from Amazon. The system is actuated by 4 FIT0458 motors, which are driven off 2 L298n motor drivers. The motors have built-in encoders for wheel odometry measurements. Also in the current sensor stack is a BNO085 9-DOF IMU. For onboard auxiliary power, 4 rechargable AA batteries were run in series. This power circuit is switched by a 5V relay module controlled by the Due's 5V power bus (i.e., when the Due switches on, the auxiliary power also comes online).

Completing the electrical design is a small breadboard circuit, which consists of a main power bus with some voltage stabilizing capacitors, voltage dividers to bring encoder ticks to levels compatible with the Due's GPIO pins (3v3), and some routing for various connections. 

The robot also has 4 ultrasonic sensors integrated into its chassis for future collision detection/last-measure obstacle avoidance - though they haven't been integrated yet due to an already-present cluttering issue. All new components were purchased off DigiKey or Amazon.
* PUT PICTURES OF THE ROBOT HERE - alternatively, nest them between paragraphs (but probably makes more sense to just have a couple at the bottom *
<br><br>

### Project status
The system is able to run untethered (controlled via teleoperation over wifi). At the moment, this is being done by remote connection into the Pi via ssh from my dev computer, but with some amount of setup this could be configured to run over ros2 through my LAN - my dev computer is running a newer ros2 version than the Pi though (Jazzy on dev computer, Humble on Pi - the Pi is running outdated Ubuntu, so it had to run outdated ros2 as well), so there may be some compatibility issues (the prospect of which put a low priority on investigating this, especially when ssh access had other advantages during development).

Wheel PIDs were written into the motor control module firmware, and seem to be performing well (at least when battery power remains high: as the power drains, the wheel PIDs seem to have trouble tracking inputs, requiring higher command inputs to achieve a given speed).
*PUT VIDEO OF TUNED WHEELS*

IMU and encoder measurements were integrated into the sensing stack, and are currently being used for closed-loop control on both the wheel speed controllers (encoder only), and the body velocity controller (encoder + IMU). Other than issues resulting from excessive wheel slip (the COTS mecanum wheels I sourced have very poor traction on most surfaces), the robot is performing reasonably well - noting trade-offs in controller tuning that I opted for to try to mitigate the wheel slip issue.
*PUT VIDEOS OF ROBOT OPERATING WITH AND WITHOUT PID CONTROLLER*

The chassis is built aside from the top shell - the design of which has been roughed out, but is awaiting final detailing before printing (it would be more of an inconvenience for platform development anyways). It will be used to mount the camera module and LiDAR scanner (see the subsection titled ["Future plans"](#future-plans) for details) and will thus need to be designed and printed before those developments can begin.
*PUT PICTURE OF CHASSIS CAD HERE, WITH LABELS ON PLANNED MOUNTING FOR LIDAR AND CAMERA)*
<br><br>

### Future plans
The long-term aim of this project was always to touch on some higher-level autonomy concepts, but given that this needed to be built towards, it was always going to be fairly low on the priority list (many basic requirements needed to be met first). While it would be nice to continue with a robot I designed and built myself, it might be best to look at some of these concepts with a more vetted platform (though any capable of handling these new requirements are likely to be very expensive, and at lower price points could carry many of the issues left to resolve with this project - if not more).

Before starting on some of the projections listed below, there is some "cleanup" of the prototype that needs to be done - see the ["Housekeeping work"](#housekeeping-work) section below. 

#### Mapping and localization (LIO SLAM)
After the core platform is working, the natural next area of focus for the project is setting up for higher-level autonomy. The addition of a LiDAR scanner would allow for the robot to perform simultaneous localization and mapping (SLAM), optionally with the integration of inertial measurement (IMU) and wheel odometry (encoder) data into the estimate. While a custom package could be written to handle this, SLAM integration could be greatly facilitated by leaning on pre-built packages such as slam_toolbox and Nav2.

However, it is likely that whatever SLAM system is designed will not be able to run onboard the Pi3B+ as originally intended - the CPU cores are already running upwards of 70% capacity under load with the current running processes, so adding the data processing, memory caching, and computations required for effective SLAM would almost certainly exceed the processing capabilities of the SBC. In order to implement SLAM with the current hardware stack, the best route may be to try to pipe LiDAR data (as well as pose estimates from other sensors) to an offboard PC, which runs the SLAM stack and returns a map with a global pose estimate to the Pi (which then dead reckons until receiving the next update). Offboarding would also allow other autonomy features (e.g., planning) to be handled by a different compute unit, allowing the onboard computer to remain free for low-level pathing (e.g., implementing sequences of velocity commands, short-range position control, obstacle avoidance, etc.) and control.
*PUT SCREENSHOT OF HTOP OUTPUT*
Depending on the feasibility of offboarding SLAM (e.g., considering how realistic the data pipelining is, and what sort of update latency would be realized), it may make more sense to just opt to purchase a better compute module - a Raspberry Pi 4B+ should be a drop-in upgrade for the features I actually care about (mounting and footprint compatibility), and would provide a far more performant processor, with purchase options that provide more built-in memory (based on [table 9 of this review](https://www.mdpi.com/2218-6581/15/8/142), the Pi 4B should be able to support LIO SLAM at 10-30Hz). The upgraded computer would also certainly be able to run Ubuntu server 24.04, which would allow me to match ros2 versions with my dev PC, and supports 5GHz WiFi, allowing for lower network latency.

#### Position control
Position control has not yet been implemented due to lack of global reference frame in the current design (it was also decidedly less important to core usability than developments done to date). Additionally, any position estimate is likely to be poor given current sensor stack and traction issues - wheel odometry can be relied on to a reasonable degree for forwards and turning motions, but any derived lateral motion estimates are completely untrustworthy, and 2nd order integration of velocity data from an IMU is certain to drift far too much to be usable.

#### Planning and autonomy
Once mapping, localization, and position control have been realized, I can look into running some basic autonomous missions (given the platform, this will probably be limited to "go from location A to B and inspect the area"). It would make sense to lean on ros2 packages such as Nav2 to implement these features.
<br><br>

### Housekeeping work
What follows is a list of "backburner" tasks that were put off in favour of more active development - the time that would need to be taken for some of these tasks, both in terms of physical workload, as well as the pursuit of knowledge required to achieve them, made them decidedly not worth doing at early stages in the project. However, now that the core platform is approaching something usable, certain of these steps should be taken for overall system performance.

Of course, when these changes are implemented, wheel and body velocity controllers may require some adjustments to match the changes (e.g., changing wheels, or if motor supply voltage, or available current changes)

#### Electrical cleanup 
The breadboard circuit, power bus, and several of the COTS components would be much better replaced with a custom PCB, both in terms of packaging, but also in terms of efficiency, as connectors and lengths of wire could be replaced with on-board routing, and circuits could be designed with intent specifically for the system. 

The L928n motor drivers which were used for simplicity's sake are fairly inefficient (50-70%), and should be replaced with either a more performance COTS driver, or, preferably, integrated into a custom PCB. This board could also have standoffs for mounting the relay (or, the relay circuit could be built into the board) to improve its packaging. Having a custom PCB would also allow many of the DuPont connectors, breadboard connections, and wire nuts to be removed for more reliable connectors (better in terms of servicing requirements, mechanical reliability, and likely to have some marginal electrical performance benefits).

It would make the most sense to design a custom PCB as a hat board for the Due (i.e., mounting using the GPIO headers on the Due, which also serve as electrical connection points), as it interfaces with the most peripherals, which in turn interface with the power system (other than interacting with the UPS, the IMU, and the Due, the computer module is fairly isolated).

Power concerns are also a high-value target area for improvement. Monitoring of the auxiliary power batteries (at least some basic monitoring, i.e., lighting an LED when it drops below some threshold) would be very beneficial from a usability perspective (at the moment, observing degraded driving performance is the main indication that batteries need charging).

Upgrades for the batteries themselves should also be investigated: at the moment, the system is being powered using rechargable AA batteries, with a capacity of 2300mAh, in series - this leads to a relatively short life between battery swaps or charges (anywhere between 15 minutes and a few hours, depending on driving performance requirements and usage). Higher-capacity rechargable batteries would make the system a lot more usable, especially where autonomous operation (where a human may not be around at a given time to swap the batteries) is concerned.

#### COTS component swap-out
**Wheels:** I used very cheap mecanum wheels sourced off Amazon for this project, whose rollers are made of a hard, slippery plastic (I assume to be ABS or HDPE), and have very poor traction. This leads to excessive slipping in the roller-axis directions, which makes holonomic movement (which was supposed to be the advantage of using the mecanum wheels) inconsistent and inefficient, in addition to having negative implications on the validity of the encoder odometry. The wheels have especially poor traction on the vinyl flooring in my apartment (they drive a little better on a rubber mat or on thin rugs), and they tend to get caught up in the carpeted flooring elsewhere in the apartment.
*attach video of robot slipping while trying to drive sideways - optionally, turn PIDs off*
I've tried my best to smooth out the controls - I've increased the integral and derivative gains in the driving direction and weighted the linear movement readings from the IMU higher than I might otherwise have in order to try to stabilize about the x=x0 operating point, so that it can at least drive (albeit slowly) in this direction, but I fear I am approaching the limit of what can practicably be done with this hardware (both in terms of sensing and state estimation with the current compute and sensor stack, and on compensating for wheel slippage).

These wheels should be swapped out with either more performant mecanum wheels (i.e., ones with better roller traction), or with regular wheels with better traction (this would require disabling lateral movement commands).

**L298n motor drivers:** As stated in the electrical section, the motor drivers used for this stage of the prototype were selected for simplicity, not performance - they are bulky and inefficient, and the headers on the board chosen require the use of DuPont connectors, which are mechanically unreliable and generally perform poorer than e.g., JST PH connectors. As mentioned, they should be swapped out with a more performance COTS driver, or preferably, integrated into a custom PCB.


#### Software cleanup
Several (small) corners were cut on the software side in order to push on key areas of development without getting bogged down writing and testing features that did not matter at the time, or reading documentation to work with certain ros2 tools. First on the list, it may be worth writing a proper URDF, rather than relying on configuration headers and launch-file based joint transform publishers to fill the gaps left in its absence. Having a URDF would also allow me to get some experience working certain ros2 tools that require one, such as ros2_control as well as simulation tools (Gazebo/RViz2/MoveIt). This seemed like overkill for this project, but seems very useful for more complicated projects with non-static joint transforms (e.g., if I wanted to work on something like an arm next).

The input handler could also use a rework or a rewrite - I would like something more reliable than the little TTY interface I wrote myself, which drops commands occasionally, particularly when the compute module's CPU gets bogged down - at the moment, this causes choppy commands to be sent, which likely would not be an issue with a handler that looks for key-pressed/key-released events. It would also be nice to have something that can handle combined inputs that weren't pre-coded, as is the case with the current input handler.

As mentioned, the chassis-mounted ultrasonic sensors still have yet to be integrated into the system (waiting on a custom PCB, as currently, the wiring is enough of a mess as-is). These sensors would be used for collision detection and/or short-term obstacle avoidance.

