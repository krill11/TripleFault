# FTC Triple Fault
This is the official code repository for FTC Team Triple Fault, and will be used for controlling versions of code for the robot built by above mentioned team, code named at this time ONYX.

***

## Getting Started
The project contains several folders, the uses of which will be detailed here. **All Code is to be written in Kotlin**. Please refer to [Kotlin Docs](https://kotlinlang.org/ "Kotlin Docs") for the
+ ### *FtcRobotController*
  - Not to be touched in most scenarios, this directory contains the base code for the visuals of the app running on the *robot controller* phone.

+ ### *TeamCode*
  - To be used for the writing of the teams own code, and contains some subdirectories, which are contained in *kotlin/org/firstinspires/ftc/teamcode/Team*, which will be detailed below.
    + #### *Basic Robots*
      - This directory contains the basic classes for the two basic drivetrains that are worth considering for FTC, Holonomic X drive, and the Mecanum drivetrain, which we are actually using

    + #### *Complex Robots*
      - This directory contains the classes for the more complex hardware of fully build robots, and the classes in here are to be extensions of the classes in the Basic Robots folder.

    + #### *Enums*
      - To be used for Enums

    + #### *OpModes*
      - This has two subdirectories, the Autonomous, and the TeleOp. Autonomous modes are modes that run in the first 30 seconds of the match, and allow the robot to perform tasks by itself. TeleOp modes are modes where a driver drives the robot using a game controller, in the last 2:30 of an FTC match.

***
