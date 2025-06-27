// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc2910.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.frc2910.robot.autos.AutoChooser;
import org.frc2910.robot.commands.ResetSwervePose;
import org.frc2910.robot.commands.TimedFlashLEDCommand;
import org.frc2910.robot.config.RobotIdentity;
import org.frc2910.robot.constants.Constants;
import org.frc2910.robot.subsystems.Superstructure;
import org.frc2910.robot.subsystems.led.LEDSubsystem;
import org.frc2910.robot.util.DummyLogReceiver;
import org.frc2910.robot.util.MacAddressUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.lang.reflect.Field;

public class Robot extends LoggedRobot {

    private static final double loopOverrunWarningTimeout = 0.2;

    private AutoChooser autoChooser;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);
        Logger.recordMetadata("Robot Identity", RobotIdentity.getIdentity().name());
        Logger.recordMetadata("Mac Address", MacAddressUtil.getMACAddress());

        switch (BuildInfo.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        SignalLogger.enableAutoLogging(false);

        Logger.addDataReceiver(new DummyLogReceiver());

        // Start AdvantageKit logger
        Logger.start();

        // Adjust loop overrun warning timeout
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(loopOverrunWarningTimeout);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
        }
        CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

        robotContainer = new RobotContainer();

        autoChooser = AutoChooser.create(robotContainer);
        Shuffleboard.getTab(Constants.operatorDashboardName)
                .add("Auto Program", autoChooser)
                .withSize(6, 3)
                .withPosition(12, 0)
                .withWidget(BuiltInWidgets.kComboBoxChooser);

        Shuffleboard.getTab(Constants.operatorDashboardName)
                .add(new ResetSwervePose(
                        new InstantCommand(() -> robotContainer
                                .getSwerveSubsystem()
                                .resetTranslationAndRotation(
                                        autoChooser.getStartingPose().orElse(new Pose2d()))) {
                            @Override
                            public boolean runsWhenDisabled() {
                                return true;
                            }
                        }.andThen(new TimedFlashLEDCommand(
                                        robotContainer.getSuperstructure(),
                                        robotContainer.getLedSubsystem(),
                                        LEDSubsystem.WantedState.DISPLAY_POSE_RESET,
                                        2.0))
                                .andThen(
                                        new InstantCommand(() -> robotContainer
                                                .getSuperstructure()
                                                .toggleHasPoseBeenSetForPrematch(true)) {
                                            @Override
                                            public boolean runsWhenDisabled() {
                                                return true;
                                            }
                                        })))
                .withSize(6, 3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.updateValues();
        Shuffleboard.update();
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(Constants.SILENCE_JOYSTICK_WARNINGS_IN_SIMULATOR);
        }
    }

    @Override
    public void disabledInit() {
        autoChooser.reset(Constants.autoChooserName);
        robotContainer.getVisionSubsystem().setThrottleValue(0);
        robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.DEFAULT_STATE);
    }

    @Override
    public void disabledPeriodic() {
        autoChooser.update();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        robotContainer.getVisionSubsystem().setThrottleValue(0);
        autoChooser.getSelectedCommand().ifPresent(CommandScheduler.getInstance()::schedule);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        robotContainer.getVisionSubsystem().setThrottleValue(0);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationPeriodic() {
        robotContainer.getSwerveSubsystem().simulationPeriodic();
    }
}
