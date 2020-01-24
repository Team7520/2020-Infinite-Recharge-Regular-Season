# 2020-Infinite-Recharge-Regular-Season

#### Welcome to the FRC 2020 season. As you know, this is where we'll be hosting our robot code. 

To be honest, I'm a little lazy, so I'm just going to put an updated version of the README from last year: 

### Programming Objectives (new for this year)
Our code will likely come with **computer vision and sensor support** by the time the season competitions begin. To elaborate, we're going to use **OpenCV** and a variety of 2D and 3D cameras to give our robot the cool, futuristic vision system it needs; also, we'll be looking to incoporate sensors like **navX and encoders** to help with autonomous functions, both during and after auton. Additionally, we need to get a firm grasp on how to use **subsystems and commands** properly in our code if we want to take our performance to the next level. 

### Required Installations
Programming in the FRC requires a ton of software/application/library downloads, and I know how much you like to install stuff, so I'm giving you a list of what you need to have on your computer before you start coding: 
- **NI Update Suite:** Because you really can't code without it, you should install this first. Going in, you should know that it comes in .zip format while requiring a password to unzip. A new version is released every year.
- **WPILib Installer:** As a team, we think this package is as much of a godsend as it was a pain to install. After you install, you're provided with libraries and API's for every piece of hardware used in the FRC as well as a special version of VS Code geared towards coding for FRC. **Don't install VS Code separately** unless you like making things hard for yourself. On the other hand, if you were part of a Labview team --and luckily we aren't one-- you would need to install Labview from NI.
- **2020 FRC Radio Configuration Utility:** Radios (robot wifi hubs) have to be configured both before and after competitions, so us programming nerds use this software to set up our radios. 
- **CTRE Phoenix Toolsuite:** Our team uses CTRE (a company) CAN devices like the VictorSPX and the TalonSRX --both of which are motor controllers-- and we can code and configure such items thanks to this handy third-party toolsuite. 
- **Kauai Labs:** Any team using a NavX or other items from Kauai Labs (another company) needs to install the corresponding libraries from Kauai Labs' website. Hooray for third-party software. 

### Quick Links
- https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html is where to go to install WPILib.
- https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/frc-game-tools.html is for the NI Update Suite and other game tools. 

### Programming Guides
Anyone who is looking to code and thinks they can dedicate some time to programming (even if they really can't) should check out the following resources: 
- https://docs.wpilib.org/en/latest/index.html gives a complete overview of what goes into making and coding a robot for the FRC.
- https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/index.html head to this page to brush up on the basics, like hardware components and using tools like the radio configurator. Speaking of hardware, check out these links:       
  - https://docs.wpilib.org/en/latest/docs/software/actuators/index.html for actuators
  - https://docs.wpilib.org/en/latest/docs/software/sensors/index.html for sensors
  - https://docs.wpilib.org/en/latest/docs/software/can-devices/index.html for CAN devices such as motor controllers.
- https://docs.wpilib.org/en/latest/docs/software/commandbased/index.html has all the information you need on advanced robot  programming.
- https://docs.wpilib.org/en/latest/docs/software/vision-processing/index.html is dedicated specifically to vision; as vision greatly boosts teams' capabilities during matches, it will be useful to have this sort of knowledge.
- https://docs.wpilib.org/en/latest/docs/software/basic-programming/git-getting-started.html doesn't have any actual programming advice, but contains some extremely helpful information on using GitHub. What a meta link. 

### Code Structure
**Just so you know, our team uses the command-based robot template as the base for our program.**  
If you're looking to pull our robot code (located in `src/main/.../robot` ), know that it can be broken up into four main components: 
```
 commands
 sensors
 subsystems
 Robot.java
```
- In the `commands` folder, you'll find a bunch of command programs, which are basically instructions to particular robot parts, such as the drivetrain, the intake, the climbing, etc. on how to execute certain actions. Sadly, they are mostly empty and/or unused. Happily, we will flesh them out this season. 
- Although we didn't employ them, you'll see two files in the `sensors` folder: `LimitSensors.java` seeks to return values (true or false) from limit switch sensors placed on our robot, and `NavX.java` has a lot of functions that are supposed to be used with feedback from the NavX on top of our RIO.  
- The files in the `subsystems` folder are essentially blueprints for our hardware, as these subsystem files outline every function we'd want our hardware to have. Our command programs aim to utilize these subsystem functions to perform actions. 
- `Robot.java` is the big boss. During matches, everything our robot does comes from Robot.java. Anything that's part of the other three components will be incorporated into this program if we find use for it. 

**Since we haven't started coding yet, be aware that this could change, especially with the addition of vision into our system.**

And that's all. As mentioned in the first paragraph, many tweaks and improvements are yet to come. With the experience we've gained last year, our performance will be relatively less sucky when we compete, and we are doing pretty well for a second-year rookie team. Here's to hoping for a functional, subsystem-using command system and finally putting some long-needed vision in our code.  
