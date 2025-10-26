# Copilot Instructions

# Description
This repository contains code for an FTC (FIRST Tech Challenge) robot using Java. The code is structured to utilize several libraries for enhanced functionality, including FTC Robot Controller, Pedro Pathing, SolversLib, Panels, and State Factory. The robot is equipped with a Limelight 3A camera for vision processing and a Gobilda Pinpoint odometry computer for localization. The Hardware is not and extensive list of all the hardware used, but only the hardware that is relevant to the code.

# FTC Game Description
DECODE is the 2025–2026 FIRST Tech Challenge game, played on a 12-foot by 12-foot field with 1-foot-high walls by two alliances—red and blue—each made up of two randomly selected teams. Robots must fit within an 18-inch cube and stay within that size limit until the End Game.

The field contains 24 purple artifacts and 12 green artifacts. Each artifact is a 5-inch diameter hard plastic ball that robots collect and score into their alliance’s goal, a 20-inch-wide and 16-inch-tall opening located at the far end of the field. Scored artifacts roll out onto a classifier ramp, a 24-inch-long sloped platform leading into the classifier box, where artifacts are held and counted. If the classifier fills up, robots can open the classifier gate to release artifacts and make space for more. Overflowing artifacts still score, but for fewer points.

At the start of each match, a pattern called a motif is randomly displayed on the Obelisk, a 24-inch-tall triangular pillar near the center of the field. The motif shows a sequence of three artifact colors—two purple artifacts and one green artifact—that determines the ideal pattern teams should aim to form in their classifier for bonus points. There are three possible motifs, depending on the position of the green artifact:

1.Green–Purple–Purple (G–P–P)

2.Purple–Green–Purple (P–G–P)

3.Purple–Purple–Green (P–P–G)

Each match lasts 2 minutes and 30 seconds, divided into three periods:

Autonomous Period (0:00–0:30): Robots operate using pre-programmed instructions and sensors, without driver control. They can detect the motif on the Obelisk, leave the starting line, and score artifacts into their goal. Bonus points are awarded if the artifacts in the classifier match the motif pattern at the end of this phase.

Tele-Operated Period (0:30–2:10): Drivers take control of their robots. Teams continue collecting and scoring artifacts, can open the classifier gate to clear it, and receive new artifacts from their human player, who feeds artifacts from the alliance’s player station. Alliances that score enough total artifacts earn an additional Ranking Point (RP) toward their tournament standings.

End Game (2:10–2:30): Robots may continue scoring but must also return to their base, a marked area near their starting position. Once inside, robots may expand vertically (beyond 18 inches tall). Points are awarded for robots that are partially or fully in their base, with bonus points if both alliance robots fit completely. Teams can also earn a Ranking Point if they move in autonomous and return to base in the End Game.

The alliance with the most total points wins the match and earns three ranking points. DECODE challenges teams to use vision systems, sensors, and precision control to “decode” the correct motif pattern and maximize scoring

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

### 2. Pedro Pathing (version 2.0.3)
- Docs: https://pedropathing.com/docs/pathing
- Focus: Autonomous path planning, path following, localization.

### 3. SolversLib (version 0.3.3)
- Docs: https://docs.seattlesolvers.com/
- Focus: Command Based programming, hardware additions, subsystems, commands, controllers.

### 4. Panels (version 1.0.9)
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