// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc2910.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc2910.robot.config.RobotConstants;
import org.frc2910.robot.config.RobotIdentity;
import org.frc2910.robot.subsystems.Superstructure;
import org.frc2910.robot.subsystems.arm.ArmSubsystem;
import org.frc2910.robot.subsystems.arm.extension.ExtensionIOTalonFX;
import org.frc2910.robot.subsystems.arm.shoulder.ShoulderIOTalonFX;
import org.frc2910.robot.subsystems.arm.wrist.WristIOTalonFX;
import org.frc2910.robot.subsystems.climber.ClimberIOPhoenix6;
import org.frc2910.robot.subsystems.climber.ClimberSubsystem;
import org.frc2910.robot.subsystems.drive.SwerveIOCTRE;
import org.frc2910.robot.subsystems.drive.SwerveSubsystem;
import org.frc2910.robot.subsystems.intake.IntakeIOPhoenix6;
import org.frc2910.robot.subsystems.intake.IntakeSubsystem;
import org.frc2910.robot.subsystems.led.LEDIOCANdle;
import org.frc2910.robot.subsystems.led.LEDSubsystem;
import org.frc2910.robot.subsystems.toggles.TogglesIOHardware;
import org.frc2910.robot.subsystems.vision.VisionIOLimelight;
import org.frc2910.robot.subsystems.vision.VisionSubsystem;
import org.frc2910.robot.util.OperatorDashboard;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final OperatorDashboard operatorDashboard;

    private final Superstructure superstructure; // = new Superstructure();

    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        RobotConstants constants = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants =
                constants.getModuleConstants();

        swerveSubsystem = new SwerveSubsystem(
                new SwerveIOCTRE(constants.getSwerveDrivetrainConstants(), constants.getModuleConstants()),
                controller,
                moduleConstants[0].SpeedAt12Volts,
                moduleConstants[0].SpeedAt12Volts
                        / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));

        intakeSubsystem = new IntakeSubsystem(new IntakeIOPhoenix6(constants.getPortConfiguration()));
        var cameraConfigs = constants.getCameraConfigurations();
        var visionIOs = new VisionIOLimelight[cameraConfigs.size()];
        for (int i = 0; i < visionIOs.length; i++) {
            visionIOs[i] = new VisionIOLimelight(cameraConfigs.get(i));
        }
        visionSubsystem = new VisionSubsystem(visionIOs);

        armSubsystem = new ArmSubsystem(
                new ExtensionIOTalonFX(constants.getPortConfiguration(), constants.getArmConfiguration()),
                new ShoulderIOTalonFX(constants.getPortConfiguration(), constants.getArmConfiguration()),
                new WristIOTalonFX(constants.getPortConfiguration(), constants.getArmConfiguration()));

        ledSubsystem = new LEDSubsystem(new LEDIOCANdle(
                constants.getPortConfiguration().candleID.getDeviceNumber(), constants.getPortConfiguration().CANBus));

        climberSubsystem = new ClimberSubsystem(new ClimberIOPhoenix6(constants.getPortConfiguration()));

        operatorDashboard = new OperatorDashboard();

        superstructure = new Superstructure(
                swerveSubsystem,
                intakeSubsystem,
                armSubsystem,
                climberSubsystem,
                ledSubsystem,
                new TogglesIOHardware(constants.getPortConfiguration()),
                operatorDashboard);

        configureBindings();
    }

    private void configureBindings() {
        new Trigger(superstructure::hasHomeButtonBeenPressed)
                .onTrue(instantCommand(() -> {
                            armSubsystem.tareAllAxesUsingButtonValues();
                            climberSubsystem.tareCarriage();
                        })
                        .alongWith(instantCommand(() ->
                                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ARM_ZEROED))));
        new Trigger(superstructure::hasSwitchValueChanged).onTrue(instantCommand(() -> {
            var neutralMode = superstructure.getNeutralMode();
            armSubsystem.setNeutralMode(neutralMode);
            climberSubsystem.setCarriageNeutralMode(neutralMode);
        }));

        controller.leftStick().onTrue(instantCommand(superstructure::toggleReefSelectionMethod));
        controller
                .povRight()
                .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.ARM_UP))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .leftBumper()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_LEFT_L4,
                        Superstructure.WantedSuperState.SCORE_L1_LEFT_TOP,
                        Superstructure.WantedSuperState.MOVE_ALGAE_TO_NET_POSITION,
                        Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_GROUND))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .leftBumper()
                .and(controller.rightStick())
                .onTrue(superstructure
                        .setStateCommand(Superstructure.WantedSuperState.SCORE_LEFT_L4)
                        .andThen(new WaitUntilCommand(() -> !superstructure.hasCoral()))
                        .andThen(
                                superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF)))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .leftTrigger()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_LEFT_L3,
                        Superstructure.WantedSuperState.SCORE_L1_LEFT_TOP,
                        Superstructure.WantedSuperState.MOVE_ALGAE_TO_PROCESSOR_POSITION,
                        Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));
        controller
                .a()
                .and(controller.rightStick())
                .onTrue(superstructure
                        .setStateCommand(Superstructure.WantedSuperState.SCORE_LEFT_L2)
                        .andThen(new WaitUntilCommand(() -> !superstructure.hasCoral()))
                        .andThen(
                                superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF)))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .a()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_LEFT_L2,
                        Superstructure.WantedSuperState.SCORE_L1_LEFT_BASE,
                        Superstructure.WantedSuperState.DEFAULT_STATE,
                        Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_MARK))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));
        controller
                .x()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_L1_MANUAL_ALIGN,
                        Superstructure.WantedSuperState.SCORE_L1_MANUAL_ALIGN,
                        Superstructure.WantedSuperState.DEFAULT_STATE,
                        Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_HP))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .rightBumper()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                        Superstructure.WantedSuperState.SCORE_L1_RIGHT_TOP,
                        Superstructure.WantedSuperState.MOVE_ALGAE_TO_NET_POSITION,
                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND_HORIZONTALLY))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .rightBumper()
                .and(controller.rightStick())
                .onTrue(superstructure
                        .setStateCommand(Superstructure.WantedSuperState.SCORE_RIGHT_L4)
                        .andThen(new WaitUntilCommand(() -> !superstructure.hasCoral()))
                        .andThen(
                                superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF)))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .rightTrigger()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_RIGHT_L3,
                        Superstructure.WantedSuperState.SCORE_L1_RIGHT_TOP,
                        Superstructure.WantedSuperState.MOVE_ALGAE_TO_PROCESSOR_POSITION,
                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .leftBumper()
                .and(controller.rightBumper())
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.MANUAL_L4,
                        Superstructure.WantedSuperState.MANUAL_L1,
                        Superstructure.WantedSuperState.SCORE_ALGAE_IN_NET,
                        Superstructure.WantedSuperState.DEFAULT_STATE))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));
        controller
                .leftTrigger()
                .and(controller.rightTrigger())
                .onTrue(Commands.either(
                        superstructure.setStateCommand(Superstructure.WantedSuperState.SCORE_ALGAE_IN_PROCESSOR),
                        superstructure.setStateCommand(Superstructure.WantedSuperState.MANUAL_L3),
                        intakeSubsystem::hasAlgae))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .b()
                .and(controller.rightStick())
                .onTrue(superstructure
                        .setStateCommand(Superstructure.WantedSuperState.SCORE_RIGHT_L2)
                        .andThen(new WaitUntilCommand(() -> !superstructure.hasCoral()))
                        .andThen(
                                superstructure.setStateCommand(Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF)))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .b()
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.SCORE_RIGHT_L2,
                        Superstructure.WantedSuperState.SCORE_L1_RIGHT_BASE,
                        Superstructure.WantedSuperState.DEFAULT_STATE,
                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_STATION_STRAIGHT))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));
        controller
                .y()
                .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.SCORE_L1_MANUAL_ALIGN))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller
                .x()
                .and(controller.y())
                .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.MANUAL_L1))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));
        controller
                .b()
                .and(controller.a())
                .onTrue(superstructure.configureButtonBinding(
                        Superstructure.WantedSuperState.MANUAL_L2,
                        Superstructure.WantedSuperState.MANUAL_L1,
                        Superstructure.WantedSuperState.DEFAULT_STATE,
                        Superstructure.WantedSuperState.DEFAULT_STATE))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

        controller.povUp().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.CLIMB));
        controller
                .povDown()
                .onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE, true));

        controller.back().onTrue(new InstantCommand(swerveSubsystem::resetRotationBasedOnAlliance));

        controller.start().onTrue(superstructure.setStateCommand(Superstructure.WantedSuperState.HOME));
    }

    public LEDSubsystem getLedSubsystem() {
        return ledSubsystem;
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    private static InstantCommand instantCommand(Runnable runnable) {
        return new InstantCommand(runnable) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }
}
