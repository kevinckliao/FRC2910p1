# 2025CompetitionRobot
FRC Team 2910's robot code for the 2025 FRC game REEFSCAPE.

### Organization
* [autos](src/main/java/org/frc2910/robot/autos) - contains autonomous programs, and framework for program selection
* [commands](src/main/java/org/frc2910/robot/commands) - contains some utility commands
* [config](src/main/java/org/frc2910/robot/config) - contains robot configuration framework and constants specific to each robot
* [constants](src/main/java/org/frc2910/robot/constants) - contains general constants for 2025 robot function 
* [subsystems](src/main/java/org/frc2910/robot/subsystems) - contains code for robot mechanisms and contains [Superstructure](src/main/java/org/frc2910/robot/subsystems/Superstructure.java) to coordinate robot action
* [util](src/main/java/org/frc2910/robot/util) - contains utility classes

### Notable features
* Fully automated coral scoring on all levels, and automated stacking of coral on L1
* Fully automated algae pickups off of the reef
* Automated climb sequence (once cage was obtained)
* Single driver/controller setup with [bindings](https://docs.google.com/spreadsheets/d/1mrNfBA8ZLrNEE5x-Iu0jWxuKyEPj8YHWPqSZqwwA-oM/edit?gid=0#gid=0) switching based on robot state 
* Vision relocalization with custom algorithm
  * Pose calculation from tag data
  * Automatic selection of optimal camera to relocalize from
* AdvantageKit logging
* CTRE swerve API 
* Custom finite state machine architecture
* Configuration allowing competition code to be run on past robots for testing purposes


### General
1. Clone this repo
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries.  (make sure you're using Java 17 or greater)
1. Run `./gradlew downloadAll` to download FRC tools (ShuffleBoard, etc.)
1. Run `./gradlew tasks` to see available options
1. Have fun!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 17 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 17 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `2025CompetitioRobot.ipr` file with IntelliJ
1. When prompted, select import Gradle build

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details.
* Run `./gradlew assemble` to build the code without running all unit tests.  Use the `--info` flag for more details.
* Run `./gradlew test` to run all the JUnit tests.
