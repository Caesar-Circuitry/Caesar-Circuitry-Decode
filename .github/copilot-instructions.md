# Copilot Instructions

# Description
This repository contains code for an FTC (FIRST Tech Challenge) robot using Java. The code is structured to utilize several libraries for enhanced functionality, including FTC Robot Controller, Pedro Pathing, SolversLib, Panels, and State Factory. The robot is equipped with a Limelight 3A camera for vision processing and a Gobilda Pinpoint odometry computer for localization. The Hardware is not and extensive list of all the hardware used, but only the hardware that is relevant to the code.

## Project Architecture
- This project is designed around a command-based architecture, leveraging the SolversLib library for structuring commands and subsystems.
### robot
    - Contains all of the logic for the robot levearging Commands and Triggers from SolversLib
### robotHardware
    - Contains all of the Subsystems for the robot and compiles their read loop write functions into a single read loop write function for the robot
### read loop write
    - Subsystems sort their code into 3 distinct methods
        - read
            - Read all of the sensors and update all of the variables
        - loop
            - Use the variables to make decisions and update other variables
        - write
            - Write to all of the actuators based on the variables
### Constants
    - All of the constants for the robot are stored in a single file for easy access and modification
### Subsystems
    - Each subsystem is responsible for a specific part of the robot, such as the drivetrain, arm, or intake must contain read loop write functions
### WSubsystem
    - A base class for all subsystems that contains common functionality and enforces the read loop write structure
### Opmodes
    - The opmodes are responsible for initializing the robot and starting the appropriate commands for autonomous or teleop modes

## Libraries to Reference

### 1. FTC Robot Controller (version 11.0)
- Docs: https://ftc-docs.firstinspires.org/en/latest/index.html
- Focus: OpModes, hardware mapping, telemetry, gamepad input, autonomous and teleop programming.

### 2. Pedro Pathing (version 2.0.2)
- Docs: https://pedropathing.com/docs/pathing
- Focus: Autonomous path planning, path following, localization.

### 3. SolversLib (version 0.3.3 Beta)
- Docs: https://docs.seattlesolvers.com/solverslib-docs-beta-0.3.3
- Focus: Command Based programming, hardware additions, subsystems, commands, controllers.

### 4. Panels (version 1.0.2)
- Docs: https://panels.bylazar.com/docs/com.bylazar.docs/
- Focus: Dashboard integration, telemetry visualization, real-time data monitoring, limelight dashboard.

### 5. State Factory (version 0.3.9)
- Docs: https://state-factory.gitbook.io/state-factory/
- Focus: State machine implementation, state transitions, event handling.

## Hardware used

### 1. Limelight 3A Camera
- Docs: https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
- Focus: Vision processing, target detection, camera configuration.
- Note: Ensure to only FTC programming documentation. other documentation is not relevant.

### 2. Gobilda Pinpoint odometry computer
- Docs: https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf
- Focus: Odometry, localization, sensor integration.