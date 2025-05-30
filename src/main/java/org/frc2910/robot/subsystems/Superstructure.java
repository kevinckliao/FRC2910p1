package org.frc2910.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.CameraConfiguration;
import org.frc2910.robot.constants.ArmPoseConstants;
import org.frc2910.robot.constants.Constants;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.subsystems.arm.ArmSubsystem;
import org.frc2910.robot.subsystems.climber.ClimberSubsystem;
import org.frc2910.robot.subsystems.drive.SwerveSubsystem;
import org.frc2910.robot.subsystems.intake.IntakeSubsystem;
import org.frc2910.robot.subsystems.led.LEDSubsystem;
import org.frc2910.robot.subsystems.toggles.TogglesIO;
import org.frc2910.robot.subsystems.toggles.TogglesIOInputsAutoLogged;
import org.frc2910.robot.util.ArmPosition;
import org.frc2910.robot.util.OperatorDashboard;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Superstructure extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ArmSubsystem armSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final OperatorDashboard operatorDashboard;

    private final TogglesIO togglesIO;
    private final TogglesIOInputsAutoLogged toggleInputs = new TogglesIOInputsAutoLogged();

    private final CommandXboxController controller = new CommandXboxController(0);

    private static final double SLOW_TELEOP_TRANSLATION_COEFFICIENT = 0.3;
    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;

    private Constants.SuperstructureConstants.AutomationLevel automationLevel =
            Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE;
    private Constants.SuperstructureConstants.ReefSelectionMethod reefSelectionMethod =
            Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION;
    private final Timer coralL1TopTimer = new Timer();

    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        ARM_UP,
        FORCE_RELOCALIZE_LEFT,
        FORCE_RELOCALIZE_RIGHT,
        MARK_PUNCH,
        INTAKE_CORAL_FROM_GROUND,
        INTAKE_CORAL_FROM_GROUND_HORIZONTALLY,
        INTAKE_CORAL_FROM_MARK,
        INTAKE_CORAL_FROM_STATION_STRAIGHT,
        SCORE_L1_MANUAL_ALIGN,
        SCORE_L1_LEFT_BASE,
        SCORE_L1_RIGHT_BASE,
        SCORE_L1_LEFT_TOP,
        SCORE_L1_RIGHT_TOP,
        SCORE_LEFT_L2,
        SCORE_LEFT_L3,
        SCORE_LEFT_L4,
        SCORE_RIGHT_L2,
        SCORE_RIGHT_L3,
        SCORE_RIGHT_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_HP,
        INTAKE_ALGAE_FROM_REEF,
        INTAKE_ALGAE_FROM_GROUND,
        INTAKE_ALGAE_FROM_MARK,
        MOVE_ALGAE_TO_NET_POSITION,
        SCORE_ALGAE_IN_NET,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        ARM_UP,
        NO_PIECE_TELEOP,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        HOLDING_ALGAE,
        FORCE_RELOCALIZE_LEFT,
        FORCE_RELOCALIZE_RIGHT,
        MARK_PUNCH,
        INTAKE_CORAL_FROM_GROUND,
        INTAKE_CORAL_FROM_GROUND_HORIZONTALLY,
        INTAKE_CORAL_FROM_MARK,
        INTAKE_CORAL_FROM_STATION,
        SCORE_TELEOP_L1_MANUAL_ALIGNMENT,
        SCORE_L1_LEFT_BASE,
        SCORE_L1_RIGHT_BASE,
        SCORE_L1_LEFT_TOP,
        SCORE_L1_RIGHT_TOP,
        SCORE_LEFT_TELEOP_L2,
        SCORE_LEFT_TELEOP_L3,
        SCORE_LEFT_TELEOP_L4,
        SCORE_RIGHT_TELEOP_L2,
        SCORE_RIGHT_TELEOP_L3,
        SCORE_RIGHT_TELEOP_L4,
        SCORE_AUTO_L1,
        SCORE_LEFT_AUTO_L2,
        SCORE_LEFT_AUTO_L3,
        SCORE_LEFT_AUTO_L4,
        SCORE_RIGHT_AUTO_L2,
        SCORE_RIGHT_AUTO_L3,
        SCORE_RIGHT_AUTO_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_HP,
        INTAKE_ALGAE_FROM_REEF,
        INTAKE_ALGAE_FROM_GROUND,
        INTAKE_ALGAE_FROM_MARK,
        MOVE_ALGAE_TO_NET_POSITION,
        SCORE_ALGAE_IN_NET,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState previousSuperState;

    private boolean hasDriveToPointSetPointBeenSet = false;
    private Constants.SuperstructureConstants.ScoringDirection scoringDirection =
            Constants.SuperstructureConstants.ScoringDirection.BACK;

    private NeutralModeValue pastSwitchValue = NeutralModeValue.Brake;

    private boolean hasArmReachedIntermediateClimbPosition = false;

    private boolean coralEjectFlag = false;

    private boolean alageEjectFlag = false;

    private boolean shouldHoldStraight = true;

    private boolean hasPoseBeenResetPrematch = false;

    private boolean allowExternalCommandToAccessLEDS = false;

    private boolean doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = false;
    private boolean hasArmReachedIntermediatePoseForReefAlgaePickup = false;
    private boolean hasDriveReachedIntermediatePoseForReefAlgaePickup = false;

    public Superstructure(
            SwerveSubsystem swerve,
            IntakeSubsystem intake,
            ArmSubsystem arm,
            ClimberSubsystem climber,
            LEDSubsystem ledSubsystem,
            TogglesIO togglesIO,
            OperatorDashboard operatorDashboard) {
        this.swerveSubsystem = swerve;
        this.intakeSubsystem = intake;
        this.armSubsystem = arm;
        this.climberSubsystem = climber;
        this.ledSubsystem = ledSubsystem;
        this.togglesIO = togglesIO;
        this.operatorDashboard = operatorDashboard;
    }

    @Override
    public void periodic() {
        pastSwitchValue = toggleInputs.switchValue;
        togglesIO.updateInputs(toggleInputs);
        Logger.processInputs("Toggles", toggleInputs);

        Logger.recordOutput("ReefSelectionMethod", reefSelectionMethod);

        if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            scoringDirection = Constants.SuperstructureConstants.ScoringDirection.BACK;
        } else if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION) {
            scoringDirection = RobotState.getInstance().getScoringDirection();
        } else {
            scoringDirection = RobotState.getInstance()
                    .getClosestRotationToFaceNearestReefFace()
                    .getSecond();
        }

        Logger.recordOutput(
                "Superstructure/ClosestRotationToReefFace",
                RobotState.getInstance()
                        .getClosestRotationToFaceNearestReefFace()
                        .getFirst());
        Logger.recordOutput("Superstructure/hasProfileBeenSet", hasDriveToPointSetPointBeenSet);

        Logger.recordOutput("Superstructure/scoringDirection", scoringDirection);

        automationLevel = operatorDashboard.getAutomationLevel();

        Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
        Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
        Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);

        currentSuperState = handStateTransitions();
        applyStates();

        if (DriverStation.isDisabled() && !allowExternalCommandToAccessLEDS) {
            if (controller.povLeft().getAsBoolean()) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_CONTROLLERS_ACTIVE);
            } else if (!armSubsystem.hasHomeCompleted() && currentSuperState != CurrentSuperState.HOME) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ARM_NOT_ZEROED);
            } else if (armSubsystem.reachedSetpoint(ArmPoseConstants.KICKSTAND_POSITION)
                    && toggleInputs.switchValue == NeutralModeValue.Brake
                    && hasCoral()
                    && climberSubsystem.isCarriageAtPosition(0.0)) {
                ledSubsystem.setWantedAction(
                        hasPoseBeenResetPrematch
                                ? LEDSubsystem.WantedState.DISPLAY_READY_FOR_MATCH
                                : LEDSubsystem.WantedState.DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH);
            } else {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ARM_ZEROED);
            }
        } else if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);
        }

        if (!climberSubsystem.isCarriageAtPosition(0.0)
                && armSubsystem.getWantedArmPose() != null
                && currentSuperState != CurrentSuperState.HOME) {
            armSubsystem.setWantedState(
                    ArmSubsystem.WantedState.MOVE_TO_POSITION,
                    new ArmPosition(
                            armSubsystem.getWantedArmPose().getExtensionLengthMeters(),
                            armSubsystem.getWantedArmPose().getShoulderAngleRot2d(),
                            Rotation2d.fromDegrees(Math.min(
                                    armSubsystem
                                            .getWantedArmPose()
                                            .getWristAngleRot2d()
                                            .getDegrees(),
                                    40.0))));
        }
    }

    private CurrentSuperState handStateTransitions() {
        previousSuperState = currentSuperState;
        switch (wantedSuperState) {
            case HOME:
                currentSuperState = CurrentSuperState.HOME;
                break;
            case MARK_PUNCH:
                currentSuperState = CurrentSuperState.MARK_PUNCH;
                break;
            case INTAKE_CORAL_FROM_GROUND:
                currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_GROUND;
                break;
            case INTAKE_CORAL_FROM_GROUND_HORIZONTALLY:
                currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_GROUND_HORIZONTALLY;
                break;
            case INTAKE_CORAL_FROM_MARK:
                currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_MARK;
                break;
            case INTAKE_CORAL_FROM_STATION_STRAIGHT:
                currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_STATION;
                break;
            case ARM_UP:
                currentSuperState = CurrentSuperState.ARM_UP;
                break;
            case DEFAULT_STATE:
                if (intakeSubsystem.hasStraightCoral() || intakeSubsystem.hasHorizontalCoral()) {
                    if (DriverStation.isAutonomous()) {
                        currentSuperState = CurrentSuperState.HOLDING_CORAL_AUTO;
                    } else {
                        currentSuperState = CurrentSuperState.HOLDING_CORAL_TELEOP;
                    }
                } else if (intakeSubsystem.hasAlgae()) {
                    currentSuperState = CurrentSuperState.HOLDING_ALGAE;
                } else {
                    if (DriverStation.isAutonomous()) {
                        currentSuperState = CurrentSuperState.NO_PIECE_AUTO;
                    } else {
                        currentSuperState = CurrentSuperState.NO_PIECE_TELEOP;
                    }
                }
                break;
            case FORCE_RELOCALIZE_LEFT:
                currentSuperState = CurrentSuperState.FORCE_RELOCALIZE_LEFT;
                break;
            case FORCE_RELOCALIZE_RIGHT:
                currentSuperState = CurrentSuperState.FORCE_RELOCALIZE_RIGHT;
                break;
            case SCORE_L1_MANUAL_ALIGN:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_AUTO_L1
                        : CurrentSuperState.SCORE_TELEOP_L1_MANUAL_ALIGNMENT;
                break;
            case SCORE_L1_LEFT_BASE:
                currentSuperState = CurrentSuperState.SCORE_L1_LEFT_BASE;
                break;
            case SCORE_L1_LEFT_TOP:
                currentSuperState = CurrentSuperState.SCORE_L1_LEFT_TOP;
                break;
            case SCORE_L1_RIGHT_BASE:
                currentSuperState = CurrentSuperState.SCORE_L1_RIGHT_BASE;
                break;
            case SCORE_L1_RIGHT_TOP:
                currentSuperState = CurrentSuperState.SCORE_L1_RIGHT_TOP;
                break;
            case SCORE_LEFT_L2:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_LEFT_AUTO_L2
                        : CurrentSuperState.SCORE_LEFT_TELEOP_L2;
                break;
            case SCORE_LEFT_L3:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_LEFT_AUTO_L3
                        : CurrentSuperState.SCORE_LEFT_TELEOP_L3;
                break;
            case SCORE_LEFT_L4:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_LEFT_AUTO_L4
                        : CurrentSuperState.SCORE_LEFT_TELEOP_L4;
                break;
            case SCORE_RIGHT_L2:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_RIGHT_AUTO_L2
                        : CurrentSuperState.SCORE_RIGHT_TELEOP_L2;
                break;
            case SCORE_RIGHT_L3:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_RIGHT_AUTO_L3
                        : CurrentSuperState.SCORE_RIGHT_TELEOP_L3;
                break;
            case SCORE_RIGHT_L4:
                currentSuperState = DriverStation.isAutonomous()
                        ? CurrentSuperState.SCORE_RIGHT_AUTO_L4
                        : CurrentSuperState.SCORE_RIGHT_TELEOP_L4;
                break;
            case MANUAL_L1:
                currentSuperState = CurrentSuperState.MANUAL_L1;
                break;
            case MANUAL_L2:
                currentSuperState = CurrentSuperState.MANUAL_L2;
                break;
            case MANUAL_L3:
                currentSuperState = CurrentSuperState.MANUAL_L3;
                break;
            case MANUAL_L4:
                currentSuperState = CurrentSuperState.MANUAL_L4;
                break;
            case INTAKE_ALGAE_FROM_MARK:
                currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_MARK;
                break;
            case INTAKE_ALGAE_FROM_HP:
                currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_HP;
                break;
            case INTAKE_ALGAE_FROM_REEF:
                currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_REEF;
                break;
            case INTAKE_ALGAE_FROM_GROUND:
                currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_GROUND;
                break;
            case MOVE_ALGAE_TO_NET_POSITION:
                currentSuperState = CurrentSuperState.MOVE_ALGAE_TO_NET_POSITION;
                break;
            case MOVE_ALGAE_TO_PROCESSOR_POSITION:
                currentSuperState = CurrentSuperState.MOVE_ALGAE_TO_PROCESSOR_POSITION;
                break;
            case SCORE_ALGAE_IN_NET:
                currentSuperState = CurrentSuperState.SCORE_ALGAE_IN_NET;
                break;
            case SCORE_ALGAE_IN_PROCESSOR:
                currentSuperState = CurrentSuperState.SCORE_ALGAE_IN_PROCESSOR;
                break;
            case CLIMB:
                currentSuperState = CurrentSuperState.CLIMB;
                break;
        }
        return currentSuperState;
    }

    private void applyStates() {
        if (previousSuperState == CurrentSuperState.CLIMB && currentSuperState != CurrentSuperState.CLIMB) {
            climberSubsystem.setWantedState(ClimberSubsystem.WantedState.STOWED);
            hasArmReachedIntermediateClimbPosition = false;
        }
        if (previousSuperState != CurrentSuperState.INTAKE_ALGAE_FROM_REEF
                && currentSuperState == CurrentSuperState.INTAKE_ALGAE_FROM_REEF
                && armSubsystem.getCurrentExtensionPositionInMeters() > Units.inchesToMeters(30.0)) {
            doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = true;
            hasArmReachedIntermediatePoseForReefAlgaePickup = false;
            hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
        } else if (currentSuperState == CurrentSuperState.INTAKE_ALGAE_FROM_REEF
                && (previousSuperState == CurrentSuperState.SCORE_LEFT_TELEOP_L2
                        || previousSuperState == CurrentSuperState.SCORE_RIGHT_TELEOP_L2)) {
            doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = true;
            hasArmReachedIntermediatePoseForReefAlgaePickup = false;
            hasDriveReachedIntermediatePoseForReefAlgaePickup = false;
        } else if (previousSuperState == CurrentSuperState.INTAKE_ALGAE_FROM_REEF
                && currentSuperState != CurrentSuperState.INTAKE_ALGAE_FROM_REEF) {
            doesArmNeedToGoToIntermediatePoseForReefAlgaePickup = false;
        }
        switch (currentSuperState) {
            case HOME:
                home();
                break;
            case INTAKE_CORAL_FROM_GROUND:
                intakeCoralFromGround();
                break;
            case INTAKE_CORAL_FROM_GROUND_HORIZONTALLY:
                intakeCoralFromGroundHorizontally();
                break;
            case ARM_UP:
                armUp();
                break;
            case MARK_PUNCH:
                markPunch();
                break;
            case INTAKE_CORAL_FROM_MARK:
                intakeCoralFromMark();
                break;
            case INTAKE_CORAL_FROM_STATION:
                intakeCoralFromStation();
                break;
            case NO_PIECE_TELEOP:
                noPiece();
                break;
            case NO_PIECE_AUTO:
                noPieceAuto();
                break;
            case HOLDING_CORAL_AUTO:
                holdingCoralAuto();
                break;
            case HOLDING_CORAL_TELEOP:
                holdingCoral();
                break;
            case HOLDING_ALGAE:
                holdingAlgae();
                break;
            case FORCE_RELOCALIZE_LEFT:
                relocalizeWithObservationToBeUsed(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case FORCE_RELOCALIZE_RIGHT:
                relocalizeWithObservationToBeUsed(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_L1_LEFT_TOP:
                scoreL1WithAutoAlign(false, Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_L1_LEFT_BASE:
                scoreL1WithAutoAlign(true, Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_L1_RIGHT_TOP:
                scoreL1WithAutoAlign(false, Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_L1_RIGHT_BASE:
                scoreL1WithAutoAlign(true, Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_TELEOP_L1_MANUAL_ALIGNMENT:
                scoreL1Teleop();
                break;
            case SCORE_LEFT_TELEOP_L2:
                scoreL2Teleop(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_LEFT_TELEOP_L3:
                scoreL3Teleop(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_LEFT_TELEOP_L4:
                scoreL4Teleop(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_RIGHT_TELEOP_L2:
                scoreL2Teleop(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_TELEOP_L3:
                scoreL3Teleop(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_TELEOP_L4:
                scoreL4Teleop(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_AUTO_L1:
                scoreL1Auto();
                break;
            case SCORE_LEFT_AUTO_L2:
                scoreL2Auto(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_LEFT_AUTO_L3:
                scoreL3Auto(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_LEFT_AUTO_L4:
                scoreL4Auto(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case SCORE_RIGHT_AUTO_L2:
                scoreL2Auto(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_AUTO_L3:
                scoreL3Auto(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE_RIGHT_AUTO_L4:
                scoreL4Auto(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case MANUAL_L4:
                ejectL4();
                break;
            case MANUAL_L3:
                ejectL3();
                break;
            case MANUAL_L2:
                ejectL2();
                break;
            case MANUAL_L1:
                ejectL1();
                break;
            case INTAKE_ALGAE_FROM_HP:
                intakeAlgaeFromHP();
                break;
            case INTAKE_ALGAE_FROM_MARK:
                intakeAlgaeFromMark();
                break;
            case INTAKE_ALGAE_FROM_REEF:
                intakeAlgaeFromReef();
                break;
            case INTAKE_ALGAE_FROM_GROUND:
                intakeAlgaeFromGround();
                break;
            case SCORE_ALGAE_IN_NET:
                scoreAlgaeNet();
                break;
            case SCORE_ALGAE_IN_PROCESSOR:
                scoreAlgaeProcessor();
                break;
            case MOVE_ALGAE_TO_NET_POSITION:
                moveAlgaeToNetPosition();
                break;
            case MOVE_ALGAE_TO_PROCESSOR_POSITION:
                moveAlgaeToProcessorPosition();
                break;
            case CLIMB:
                automatedCLimb();
                break;
            case STOPPED:
                stopped();
                break;
        }
    }

    private void home() {
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ZERO_ACTION);

        if (armSubsystem.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.IDLE);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.HOME);
        }

        if (climberSubsystem.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
            climberSubsystem.setWantedState(ClimberSubsystem.WantedState.IDLE);
        } else {
            climberSubsystem.setWantedState(ClimberSubsystem.WantedState.HOME);
        }

        if (armSubsystem.hasHomeCompleted()
                && climberSubsystem.hasHomeCompleted()
                && previousSuperState == CurrentSuperState.HOME) {
            setWantedSuperState(WantedSuperState.DEFAULT_STATE);
        }
    }

    private void stopped() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.IDLE);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OFF);
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.IDLE);
        ledSubsystem.setWantedAction(
                armSubsystem.hasHomeCompleted()
                        ? LEDSubsystem.WantedState.DISPLAY_ROBOT_ARM_ZEROED
                        : LEDSubsystem.WantedState.DISPLAY_ROBOT_ARM_NOT_ZEROED);
    }

    private void holdingAlgae() {
        alageEjectFlag = false;
        hasDriveToPointSetPointBeenSet = false;
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_ALGAE);

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_ALGAE);
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.STOWED);

        intakeSubsystem.setWantedState(
                armSubsystem.reachedSetpoint()
                        ? IntakeSubsystem.WantedState.HOLD_ALGAE
                        : IntakeSubsystem.WantedState.HOLD_ALGAE_HARDER);
    }

    private void holdingCoral() {
        coralL1TopTimer.stop();
        hasDriveToPointSetPointBeenSet = false;
        coralEjectFlag = false;
        armSubsystem.setWantedState(
                ArmSubsystem.WantedState.MOVE_TO_POSITION,
                shouldHoldStraight ? ArmPoseConstants.HOLD_CORAL_TELEOP : ArmPoseConstants.L1_CORAL_FRONT);
        intakeSubsystem.setWantedState(
                shouldHoldStraight
                        ? IntakeSubsystem.WantedState.HOLD_CORAL_STRAIGHT
                        : IntakeSubsystem.WantedState.HOLD_CORAL_HORIZONTAL);
        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerveSubsystem.setRotationVelocityCoefficient(shouldHoldStraight ? 1.0 : 0.6);
        if (!allowExternalCommandToAccessLEDS) {
            ledSubsystem.setWantedAction(
                    (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE)
                            ? LEDSubsystem.WantedState.DISPLAY_HOLDING_CORAL_POSE_BASED
                            : LEDSubsystem.WantedState.DISPLAY_HOLDING_CORAL_ROTATION_BASED);
        }
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.STOWED);
    }

    private void holdingCoralAuto() {
        coralEjectFlag = false;
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_CORAL_AUTO);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOLD_CORAL_STRAIGHT);
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.REJECT);
    }

    private void armUp() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.ARM_UP);
        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
    }

    private void noPiece() {
        coralL1TopTimer.stop();
        hasDriveToPointSetPointBeenSet = false;
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.STOWED);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OFF);
        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(REGULAR_TELEOP_TRANSLATION_COEFFICIENT);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);
        if (!allowExternalCommandToAccessLEDS) {
            if (!armSubsystem.hasHomeCompleted()) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ARM_NOT_ZEROED);
            } else if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_POSE_BASED);
            } else if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROTATION_BASED);
            }
        }

        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.STOWED);
    }

    private void noPieceAuto() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.STOWED);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OFF);
        climberSubsystem.setWantedState(ClimberSubsystem.WantedState.REJECT);
    }

    private void relocalizeWithObservationToBeUsed(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        Logger.recordOutput("Superstructure/IsRelocalizingInAuto", false);
        var observation = getValidTagObservationForCoralScoring(scoringSide);
        if (observation.isPresent()) {
            Logger.recordOutput("Superstructure/IsRelocalizingInAuto", true);
            swerveSubsystem.resetTranslation(observation.get().robotPoseFromCamera());
        }
    }

    private void intakeCoralFromGround() {
        coralEjectFlag = false;
        shouldHoldStraight = true;
        if (DriverStation.isAutonomous()
                && armSubsystem.getCurrentExtensionPositionInMeters() > Units.inchesToMeters(30.0)
                && armSubsystem.getCurrentWristPosition().getDegrees() > 125.0) {
            var targetArmPose = ArmPoseConstants.GROUND_CORAL_STRAIGHT;
            var clampedArmPose = new ArmPosition(
                    targetArmPose.getExtensionLengthMeters(),
                    targetArmPose.getShoulderAngleRot2d(),
                    Rotation2d.fromDegrees(125.0));
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, clampedArmPose);
        } else if (intakeSubsystem.isBackCanRangeTripped()) {
            if (DriverStation.isAutonomous()) {
                armSubsystem.setWantedState(
                        ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_CORAL_AUTO);
            } else {
                armSubsystem.setWantedState(
                        ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_CORAL_TELEOP);
            }
        } else {
            armSubsystem.setWantedState(
                    ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.GROUND_CORAL_STRAIGHT);
        }

        if (armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(10.0)) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_CORAL_STRAIGHT);
        }

        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_CORAL);
    }

    private void intakeCoralFromGroundHorizontally() {
        coralL1TopTimer.stop();
        coralEjectFlag = false;
        shouldHoldStraight = false;
        if (intakeSubsystem.hasHorizontalCoral()) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L1_CORAL_FRONT);
        } else {
            armSubsystem.setWantedState(
                    ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.GROUND_CORAL_HORIZONTAL);
        }
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_CORAL_HORIZONTAL);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_CORAL_HORIZONTAL);
    }

    private void markPunch() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.MARK_PUNCH);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OFF);
    }

    private void intakeCoralFromMark() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.MARK_CORAL);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_CORAL_STRAIGHT);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_CORAL);
    }

    private void intakeCoralFromStation() {
        coralEjectFlag = false;
        if (DriverStation.isAutonomous()) {
            var targetArmPose = ArmPoseConstants.STATION_CORAL;
            if (armSubsystem.getCurrentExtensionPositionInMeters() > Units.inchesToMeters(30.0)) {
                var clampedArmPose = new ArmPosition(
                        targetArmPose.getExtensionLengthMeters(),
                        targetArmPose.getShoulderAngleRot2d(),
                        Rotation2d.fromDegrees(125.0));
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, clampedArmPose);
            } else {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, targetArmPose);
            }

            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_CORAL);
            if (intakeSubsystem.isBackCanRangeTripped()
                    && armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(30.0)) {
                armSubsystem.setWantedState(
                        ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_CORAL_AUTO);
            }
            if (armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(10.0)) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_CORAL_FROM_STATION);
            }
        } else {
            if (intakeSubsystem.hasStraightCoral()) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_CORAL_POSE_BASED);
                swerveSubsystem.setState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
            } else if (RobotState.getInstance()
                            .getRobotPoseFromSwerveDriveOdometry()
                            .getRotation()
                            .getDegrees()
                    >= 0) {
                var angleToSnapTo = FieldConstants.isBlueAlliance() ? 126.0 : 54.0;
                swerveSubsystem.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
            } else {
                var angleToSnapTo = FieldConstants.isBlueAlliance() ? -126.0 : -54.0;
                swerveSubsystem.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
            }
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.STATION_CORAL);
            if (armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(10.0)) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_CORAL_FROM_STATION);
            }
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_CORAL);
        }
    }

    private void intakeAlgaeFromReef() {
        var levelMap = FieldConstants.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAlgae
                : Constants.ReefConstants.redAllianceAlgae;

        var location = levelMap.get(
                reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                        ? RobotState.getInstance().getClosest60Degrees()
                        : RobotState.getInstance()
                                .getClosestRotationToFaceNearestReefFace()
                                .getFirst());
        var level = scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT
                ? location.FRONT
                : location.BACK;
        var targetArmPose = ArmPoseConstants.L2_REEF_ALGAE_BACK;

        if (doesArmNeedToGoToIntermediatePoseForReefAlgaePickup && !hasArmReachedIntermediatePoseForReefAlgaePickup) {
            if (level == Constants.ReefConstants.AlgaeIntakeLocation.L2) {
                targetArmPose = scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT
                        ? ArmPoseConstants.L2_REEF_ALGAE_FRONT_INTERMEDIATE
                        : ArmPoseConstants.L2_REEF_ALGAE_BACK_INTERMEDIATE;
            } else {
                targetArmPose = scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT
                        ? ArmPoseConstants.L3_REEF_ALGAE_FRONT_INTERMEDIATE
                        : ArmPoseConstants.L3_REEF_ALGAE_BACK_INTERMEDIATE;
            }
            if (armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(30.0)) {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, targetArmPose);
                if (armSubsystem.reachedSetpoint()) {
                    hasArmReachedIntermediatePoseForReefAlgaePickup = true;
                }
            } else {
                if (armSubsystem.getCurrentWristPosition().getDegrees() > 125.0) {
                    var clampedArmPose = new ArmPosition(
                            targetArmPose.getExtensionLengthMeters(),
                            targetArmPose.getShoulderAngleRot2d(),
                            Rotation2d.fromDegrees(125.0));
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, clampedArmPose);
                } else {
                    armSubsystem.setOnlyExtensionAndShoulder(ArmSubsystem.WantedState.MOVE_TO_POSITION, targetArmPose);
                }
            }
        } else {
            if (level == Constants.ReefConstants.AlgaeIntakeLocation.L2) {
                targetArmPose = scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT
                        ? ArmPoseConstants.L2_REEF_ALGAE_FRONT
                        : ArmPoseConstants.L2_REEF_ALGAE_BACK;
            } else {
                targetArmPose = scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT
                        ? ArmPoseConstants.L3_REEF_ALGAE_FRONT
                        : ArmPoseConstants.L3_REEF_ALGAE_BACK;
            }
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, targetArmPose);
        }
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_ALGAE);

        var observation = getValidTagObservationForReefIntaking();
        var angleToIdMap = FieldConstants.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAngleToTagIDsMap
                : Constants.ReefConstants.redAllianceAngleToTagIDsMap;
        var ids = angleToIdMap.get(
                reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                        ? RobotState.getInstance().getClosest60Degrees()
                        : RobotState.getInstance()
                                .getClosestRotationToFaceNearestReefFace()
                                .getFirst());
        var id = scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT
                ? ids.FRONT_ID
                : ids.BACK_ID;

        if (doesArmNeedToGoToIntermediatePoseForReefAlgaePickup && !hasDriveReachedIntermediatePoseForReefAlgaePickup) {
            swerveSubsystem.setDesiredPoseForDriveToPoint(getIntermediatePointToDriveToForAlgaeIntaking(id));
            if (swerveSubsystem.isAtDriveToPointSetpoint()) {
                hasDriveReachedIntermediatePoseForReefAlgaePickup = true;
            }
        } else if (intakeSubsystem.hasAlgae()) {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_ALGAE);
            swerveSubsystem.setDesiredPoseForDriveToPoint(getBackoutPointToDriveToForAlgaeIntaking(id));
        } else if (observation.isPresent()) {
            var observationToBeUsed = observation.get();
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);
            swerveSubsystem.resetTranslation(observationToBeUsed.robotPoseFromCamera());
            if (!armSubsystem.reachedSetpoint()) {
                swerveSubsystem.setDesiredPoseForDriveToPoint(getIntermediatePointToDriveToForAlgaeIntaking(id));
            } else {
                swerveSubsystem.setDesiredPoseForDriveToPoint(getDesiredPointToDriveToForAlgaeIntaking(id));
            }
        } else {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.TAG_NOT_SEEN);
            if (!armSubsystem.reachedSetpoint()) {
                swerveSubsystem.setDesiredPoseForDriveToPoint(getIntermediatePointToDriveToForAlgaeIntaking(id));
            } else {
                swerveSubsystem.setDesiredPoseForDriveToPoint(getDesiredPointToDriveToForAlgaeIntaking(id));
            }
        }
    }

    private void intakeAlgaeFromGround() {
        if (intakeSubsystem.hasAlgae()) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_ALGAE);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.GROUND_ALGAE);
        }
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_ALGAE);
    }

    private void intakeAlgaeFromHP() {
        var rotation = FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
        if (RobotState.getInstance()
                        .getRobotPoseFromSwerveDriveOdometry()
                        .getRotation()
                        .getDegrees()
                > 0) {
            rotation = Rotation2d.kCCW_90deg;
        } else {
            rotation = Rotation2d.kCW_90deg;
        }
        swerveSubsystem.setTargetRotation(rotation);
        if (intakeSubsystem.hasAlgae()) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_ALGAE);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HP_ALGAE);
        }
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_ALGAE);
    }

    private void intakeAlgaeFromMark() {
        if (intakeSubsystem.hasAlgae()) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HOLD_ALGAE);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.MARK_ALGAE);
        }
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.COLLECT_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_INTAKING_ALGAE);
    }

    private void scoreL1WithAutoAlign(
            boolean isScoringBase, Constants.SuperstructureConstants.ScoringSide scoringSide) {
        var angleToIdMap = FieldConstants.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAngleToTagIDsMap
                : Constants.ReefConstants.redAllianceAngleToTagIDsMap;

        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOLD_CORAL_HORIZONTAL);
        armSubsystem.setWantedState(
                ArmSubsystem.WantedState.MOVE_TO_POSITION,
                isScoringBase ? ArmPoseConstants.L1_CORAL_BASE : ArmPoseConstants.L1_CORAL_TOP);

        driveToScoringPoseAndReturnIfObservationIsPresent(scoringSide, false, false, isScoringBase);

        var ids = angleToIdMap.get(
                reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                        ? RobotState.getInstance().getClosest60Degrees()
                        : RobotState.getInstance()
                                .getClosestRotationToFaceNearestReefFace()
                                .getFirst());

        if (isReadyToEjectInTeleopPeriod()) {
            coralEjectFlag = true;
        }

        var ejectionTime = isScoringBase ? 0.1 : 0.05;

        if (coralEjectFlag) {
            if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_L1);
            }
            if (!intakeSubsystem.hasHorizontalCoral()) {
                if (!coralL1TopTimer.isRunning()) {
                    coralL1TopTimer.restart();
                } else if (coralL1TopTimer.get() > ejectionTime) {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OFF);

                    if (coralL1TopTimer.get() > 0.2) {
                        swerveSubsystem.setDesiredPoseForDriveToPoint(FieldConstants.getBackoutPointToForL1Scoring(
                                ids.FRONT_ID,
                                isScoringBase,
                                scoringSide,
                                Constants.SuperstructureConstants.ScoringDirection.FRONT));
                    }
                }
            }
        }
    }

    private void ejectL1() {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_L1);
    }

    private void ejectL2() {
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2_CORAL_FRONT);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2_CORAL_BACK);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
        }
    }

    private void ejectL3() {
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_FRONT);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_BACK);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
        }
    }

    private void ejectL4() {
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_FRONT);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_BACK);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
        }
    }

    private void scoreL1Teleop() {
        if (intakeSubsystem.hasHorizontalCoral()) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOLD_CORAL_HORIZONTAL);
        } else {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
        }

        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L1_CORAL_FRONT);

        var validTagId = reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                ? RobotState.getInstance().getValidTagIDsFromClosest60DegreeRotation().FRONT_ID
                : RobotState.getInstance().getClosestTagId();

        for (var observation : RobotState.getInstance().getAprilTagObservations()) {
            if (observation.tagId() == validTagId) {
                swerveSubsystem.resetTranslation(observation.robotPoseFromCamera());
            }
        }

        swerveSubsystem.setTargetRotation(
                reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                        ? RobotState.getInstance().getClosest60Degrees()
                        : RobotState.getInstance()
                                .getClosestRotationToFaceNearestReefFace(true)
                                .getFirst());
    }

    private void scoreL1Auto() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L1_CORAL_FRONT);
    }

    private void scoreL2Teleop(Constants.SuperstructureConstants.ScoringSide scoringSide) {

        var isObservationPresent = driveToScoringPoseAndReturnIfObservationIsPresent(scoringSide);

        if (isObservationPresent) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2_CORAL_FRONT);
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_FORWARD);

            } else {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2_CORAL_BACK);
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
            }
        }

        if (isReadyToEjectInTeleopPeriod() && isObservationPresent) {
            coralEjectFlag = true;
        }

        if (coralEjectFlag) {
            if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
                if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
                } else {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
                }
            }
            if (!hasCoral()) {
                setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            }
        }
    }

    private void scoreL2Auto(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2_CORAL_FRONT);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2_CORAL_BACK);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
        }

        if (armSubsystem.reachedSetpoint()
                && scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_FORWARD);
        }

        if (isReadyToEjectInAutoPeriod()) {
            coralEjectFlag = true;
        }

        if (coralEjectFlag) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
            } else {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
            }
        }
        relocalizeWithObservationToBeUsed(scoringSide);
    }

    private void scoreL3Teleop(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        var isObservationPresent = driveToScoringPoseAndReturnIfObservationIsPresent(scoringSide);
        if (isObservationPresent) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_FRONT);
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_FORWARD);
            } else {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_BACK);
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
            }
        }

        if (isReadyToEjectInTeleopPeriod() && isObservationPresent) {
            coralEjectFlag = true;
        }

        if (coralEjectFlag) {
            if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
                if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
                } else {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
                }
            }
            if (!hasCoral()) {
                setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            }
        }
    }

    private void scoreL3Auto(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_FRONT);
        } else {
            if (armSubsystem.getCurrentShoulderPosition().getDegrees() < 30.0) {
                armSubsystem.setOnlyExtensionAndShoulder(
                        ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_BACK);
            } else {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3_CORAL_BACK);
            }
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
        }

        if (armSubsystem.reachedSetpoint()
                && scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_FORWARD);
        }

        if (isReadyToEjectInAutoPeriod()) {
            coralEjectFlag = true;
        }

        if (coralEjectFlag) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
            } else {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
            }
        }

        relocalizeWithObservationToBeUsed(scoringSide);
    }

    private void scoreL4Teleop(Constants.SuperstructureConstants.ScoringSide scoringSide) {

        var isObservationPresent = false;
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.BACK
                && armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(30.0)) {
            isObservationPresent = driveToScoringPoseAndReturnIfObservationIsPresent(scoringSide, true, true);
        } else {
            isObservationPresent = driveToScoringPoseAndReturnIfObservationIsPresent(scoringSide);
        }
        if (isObservationPresent) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_FRONT);
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_FORWARD);

            } else {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_BACK);
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
            }
        }

        if (isReadyToEjectInTeleopPeriod() && isObservationPresent) {
            coralEjectFlag = true;
        }

        if (coralEjectFlag) {
            if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
                if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
                } else {
                    intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
                }
            }
            if (!hasCoral()) {
                setWantedSuperState(WantedSuperState.DEFAULT_STATE);
            }
        }
    }

    private void scoreL4Auto(Constants.SuperstructureConstants.ScoringSide scoringSide) {
        if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_FRONT);
        } else {
            if (armSubsystem.getCurrentShoulderPosition().getDegrees() < 30.0) {
                armSubsystem.setOnlyExtensionAndShoulder(
                        ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_BACK);
            } else {
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4_CORAL_BACK);
            }
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_BACKWARD);
        }

        if (armSubsystem.reachedSetpoint()) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.INDEX_CORAL_FORWARD);
            }
        }

        if (isReadyToEjectInAutoPeriod()) {
            coralEjectFlag = true;
        }

        if (coralEjectFlag) {
            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_BACKWARDS);
            } else {
                intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_CORAL_FORWARDS);
            }
        }

        relocalizeWithObservationToBeUsed(scoringSide);
    }

    private void moveAlgaeToNetPosition() {
        var rotation = FieldConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
        if (Math.abs(RobotState.getInstance()
                        .getRobotPoseFromSwerveDriveOdometry()
                        .getRotation()
                        .getDegrees())
                > 90) {
            rotation = Rotation2d.k180deg;
        } else {
            rotation = Rotation2d.kZero;
        }

        swerveSubsystem.setTargetRotation(rotation);

        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOLD_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ALIGN_TO_BARGE);
        swerveSubsystem.setTeleopVelocityCoefficient(0.4);

        if (swerveSubsystem.isAtDesiredRotation()) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.NET_ALGAE_PREP);
        }

        if (alageEjectFlag) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_ALGAE);
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_SCORE_BARGE);
            swerveSubsystem.setTeleopVelocityCoefficient(0.0);
        }
    }

    private void moveAlgaeToProcessorPosition() {
        var rotation = FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
        if (RobotState.getInstance()
                        .getRobotPoseFromSwerveDriveOdometry()
                        .getRotation()
                        .getDegrees()
                > 0) {
            rotation = Rotation2d.kCCW_90deg;
        } else {
            rotation = Rotation2d.kCW_90deg;
        }
        swerveSubsystem.setTargetRotation(rotation);
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.PROCESSOR_ALGAE);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOLD_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ALIGN_TO_PROCESSOR);
    }

    private void scoreAlgaeNet() {
        swerveSubsystem.setTeleopVelocityCoefficient(0.0);
        if (swerveSubsystem.isAtDesiredRotation()) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.NET_ALGAE);
        }
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_ALGAE);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_SCORE_BARGE);
    }

    private void scoreAlgaeProcessor() {
        var rotation = FieldConstants.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
        if (RobotState.getInstance()
                        .getRobotPoseFromSwerveDriveOdometry()
                        .getRotation()
                        .getDegrees()
                > 0) {
            rotation = Rotation2d.kCCW_90deg;
        } else {
            rotation = Rotation2d.kCW_90deg;
        }
        swerveSubsystem.setTargetRotation(rotation);
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.PROCESSOR_ALGAE);
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.EJECT_ALGAE_PROCESSOR);
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_SCORE_PROCESSOR);
    }

    private void automatedCLimb() {
        if (controller.rightTrigger().getAsBoolean()) {
            swerveSubsystem.setTeleopVelocityCoefficient(0.2);
        } else {
            swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        }

        if (climberSubsystem.getHasCage()) {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_CLIMB_AUTOMATED);
            if (armSubsystem.getCurrentShoulderPosition().getDegrees() < 0.0) {
                hasArmReachedIntermediateClimbPosition = true;
            }
            if (hasArmReachedIntermediateClimbPosition) {
                climberSubsystem.setDesiredCarriagePosition(Units.inchesToMeters(0.0));
                swerveSubsystem.setTeleopVelocityCoefficient(0.0);
                armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.CLIMB_LOCK);
            } else {
                armSubsystem.setWantedState(
                        ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.CLIMB_INTERMEDIATE_POSE);
                climberSubsystem.setIntakeAndMoveState(
                        Constants.ClimberConstants.INTAKE_MOTOR_VOLTAGE, Units.inchesToMeters(0.0));
            }
        } else {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_CLIMB_ALIGN);
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.CLIMB_COLLECT);
            climberSubsystem.setDesiredCarriagePosition(Constants.ClimberConstants.CLIMBER_CARRIAGE_INTAKE_POSITION);
            climberSubsystem.setWantedState(ClimberSubsystem.WantedState.INTAKE);
        }
    }

    public boolean isReadyToEjectInAutoPeriod() {
        return armSubsystem.reachedSetpoint() && swerveSubsystem.isAtEndOfChoreoTrajectoryOrDriveToPoint();
    }

    public boolean isReadyToEjectInTeleopPeriod() {
        return swerveSubsystem.isAtDriveToPointSetpoint()
                && swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(2.0))
                && armSubsystem.reachedSetpoint();
    }

    public boolean driveToScoringPoseAndReturnIfObservationIsPresent(
            Constants.SuperstructureConstants.ScoringSide scoringSide,
            boolean isScoringL4OverTheBackAndArmIsNotAtPose,
            boolean isScoringReefCoral) {
        return driveToScoringPoseAndReturnIfObservationIsPresent(
                scoringSide, isScoringL4OverTheBackAndArmIsNotAtPose, isScoringReefCoral, false);
    }

    public boolean driveToScoringPoseAndReturnIfObservationIsPresent(
            Constants.SuperstructureConstants.ScoringSide scoringSide) {
        return driveToScoringPoseAndReturnIfObservationIsPresent(scoringSide, false, true, false);
    }

    public boolean driveToScoringPoseAndReturnIfObservationIsPresent(
            Constants.SuperstructureConstants.ScoringSide scoringSide,
            boolean isScoringL4OverTheBackAndArmIsNotAtPose,
            boolean isScoringReefCoral,
            boolean isScoringBase) {
        var observationToBeUsed = getValidTagObservationForCoralScoring(scoringSide);

        if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE) {
            if (observationToBeUsed.isPresent()) {
                var observation = observationToBeUsed.get();
                swerveSubsystem.resetTranslation(observation.robotPoseFromCamera());
            }

            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);

            Pose2d desiredPoseToDriveTo = isScoringReefCoral
                    ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                            RobotState.getInstance().getClosestTagId(), scoringSide, scoringDirection)
                    : FieldConstants.getDesiredPointToDriveToForL1Scoring(
                            RobotState.getInstance().getClosestTagId(), isScoringBase, scoringSide, scoringDirection);

            if (isScoringL4OverTheBackAndArmIsNotAtPose
                    || !swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(5.0))) {
                desiredPoseToDriveTo = new Pose2d(
                        desiredPoseToDriveTo
                                .getTranslation()
                                .plus(new Translation2d(
                                        Units.inchesToMeters(6.0),
                                        FieldConstants.getTagPose(
                                                        RobotState.getInstance().getClosestTagId())
                                                .getRotation()
                                                .toRotation2d())),
                        desiredPoseToDriveTo.getRotation());
                swerveSubsystem.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
                Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
                return true;
            } else {
                swerveSubsystem.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
                hasDriveToPointSetPointBeenSet = true;
                Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
                return true;
            }
        } else {
            if (observationToBeUsed.isPresent()) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);
                var observation = observationToBeUsed.get();
                swerveSubsystem.resetTranslation(observation.robotPoseFromCamera());

                Pose2d desiredPoseToDriveTo = isScoringReefCoral
                        ? FieldConstants.getDesiredFinalScoringPoseForCoral(
                                RobotState.getInstance().getClosestTagId(), scoringSide, scoringDirection)
                        : FieldConstants.getDesiredPointToDriveToForL1Scoring(
                                RobotState.getInstance().getClosestTagId(),
                                isScoringBase,
                                scoringSide,
                                scoringDirection);

                Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
                if (isScoringL4OverTheBackAndArmIsNotAtPose
                        || !swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(5.0))) {
                    desiredPoseToDriveTo = new Pose2d(
                            desiredPoseToDriveTo
                                    .getTranslation()
                                    .plus(new Translation2d(
                                            Units.inchesToMeters(6.0),
                                            FieldConstants.getTagPose(observation.tagId())
                                                    .getRotation()
                                                    .toRotation2d())),
                            desiredPoseToDriveTo.getRotation());
                    swerveSubsystem.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
                    Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
                    return true;
                } else {
                    swerveSubsystem.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
                    hasDriveToPointSetPointBeenSet = true;
                    Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
                    return true;
                }
            } else if (!hasDriveToPointSetPointBeenSet) {
                ledSubsystem.setWantedAction(LEDSubsystem.WantedState.TAG_NOT_SEEN);
                if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION) {
                    swerveSubsystem.setTargetRotation(RobotState.getInstance().getClosest60Degrees());
                } else if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE) {
                    swerveSubsystem.setTargetRotation(RobotState.getInstance()
                            .getClosestRotationToFaceNearestReefFace()
                            .getFirst());
                }
                return false;
            } else {
                return false;
            }
        }
    }

    public Optional<RobotState.AprilTagObservation> getValidTagObservationForReefIntaking() {
        Optional<RobotState.AprilTagObservation> observationToBeUsed = Optional.empty();
        var validTagIds = RobotState.getInstance().getValidTagIDsFromClosest60DegreeRotation();
        for (var observation : RobotState.getInstance().getAprilTagObservations()) {
            if (observation.tagId() == validTagIds.FRONT_ID
                    && (observation.location() == CameraConfiguration.Location.FRONT_LEFT
                            || observation.location() == CameraConfiguration.Location.FRONT_RIGHT)) {
                if (observationToBeUsed.isEmpty()) {
                    observationToBeUsed = Optional.of(observation);
                }
            } else if (observation.tagId() == validTagIds.BACK_ID
                    && (observation.location() == CameraConfiguration.Location.BACK_LEFT
                            || observation.location() == CameraConfiguration.Location.BACK_RIGHT)) {
                if (observationToBeUsed.isEmpty()) {
                    observationToBeUsed = Optional.of(observation);
                }
            }
        }

        Logger.recordOutput("Superstructure/AprilTagObservationToBeUsed/isPresent", observationToBeUsed.isPresent());
        if (observationToBeUsed.isPresent()) {
            Logger.recordOutput(
                    "Superstructure/AprilTagObservationToBeUsed/robotPoseFromCamera",
                    observationToBeUsed.get().robotPoseFromCamera());
            Logger.recordOutput(
                    "Superstructure/AprilTagObservationToBeUsed/location",
                    observationToBeUsed.get().location());
            Logger.recordOutput(
                    "Superstructure/AprilTagObservationToBeUsed/tagId",
                    observationToBeUsed.get().tagId());
            Logger.recordOutput(
                    "Superstructure/AprilTagObservationToBeUsed/cameraName",
                    observationToBeUsed.get().cameraName());
        }

        return observationToBeUsed;
    }

    public Optional<RobotState.AprilTagObservation> getValidTagObservationForCoralScoring(
            Constants.SuperstructureConstants.ScoringSide scoringSide) {
        Optional<RobotState.AprilTagObservation> observationToBeUsed = Optional.empty();
        Pair<CameraConfiguration.Location, CameraConfiguration.Location> validLocations =
                scoringSide == Constants.SuperstructureConstants.ScoringSide.RIGHT
                        ? Pair.of(CameraConfiguration.Location.FRONT_LEFT, CameraConfiguration.Location.BACK_RIGHT)
                        : Pair.of(CameraConfiguration.Location.FRONT_RIGHT, CameraConfiguration.Location.BACK_LEFT);

        if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                || DriverStation.isAutonomous()) {

            var validTagIds = RobotState.getInstance().getValidTagIDsFromClosest60DegreeRotation();

            for (var observation : RobotState.getInstance().getAprilTagObservations()) {
                if (observation.tagId() == validTagIds.FRONT_ID
                        && (observation.location() == CameraConfiguration.Location.FRONT_LEFT
                                || observation.location() == CameraConfiguration.Location.FRONT_RIGHT)) {
                    if (observationToBeUsed.isEmpty() || observation.location() == validLocations.getFirst()) {
                        observationToBeUsed = Optional.of(observation);
                    }
                } else if (observation.tagId() == validTagIds.BACK_ID
                        && (observation.location() == CameraConfiguration.Location.BACK_LEFT
                                || observation.location() == CameraConfiguration.Location.BACK_RIGHT)) {
                    if (observationToBeUsed.isEmpty() || observation.location() == validLocations.getSecond()) {
                        observationToBeUsed = Optional.of(observation);
                    }
                }
            }

            Logger.recordOutput(
                    "Superstructure/AprilTagObservationToBeUsed/isPresent", observationToBeUsed.isPresent());
            if (observationToBeUsed.isPresent()) {
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/robotPoseFromCamera",
                        observationToBeUsed.get().robotPoseFromCamera());
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/location",
                        observationToBeUsed.get().location());
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/tagId",
                        observationToBeUsed.get().tagId());
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/cameraName",
                        observationToBeUsed.get().cameraName());
            }
        } else {
            var desiredId = RobotState.getInstance().getClosestTagId();
            for (var observation : RobotState.getInstance().getAprilTagObservations()) {
                if (observation.tagId() == desiredId) {
                    if (observation.location() == validLocations.getFirst()
                            || observation.location() == validLocations.getSecond()) {
                        observationToBeUsed = Optional.of(observation);
                    }
                }
            }
            Logger.recordOutput(
                    "Superstructure/AprilTagObservationToBeUsed/isPresent", observationToBeUsed.isPresent());
            if (observationToBeUsed.isPresent()) {
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/robotPoseFromCamera",
                        observationToBeUsed.get().robotPoseFromCamera());
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/location",
                        observationToBeUsed.get().location());
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/tagId",
                        observationToBeUsed.get().tagId());
                Logger.recordOutput(
                        "Superstructure/AprilTagObservationToBeUsed/cameraName",
                        observationToBeUsed.get().cameraName());
            }
        }
        return observationToBeUsed;
    }

    public Pose2d getBackoutPointToDriveToForAlgaeIntaking(int tagID) {
        if (tagID >= 1 && tagID <= 22) {
            Pose2d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
            double xOffset = Units.inchesToMeters(
                    Constants.SuperstructureConstants.X_OFFSET_FROM_TAG_FOR_BACKOUT_INTAKING_ALGAE_INCHES);

            Translation2d offsetFromTag = new Translation2d(xOffset, 0);

            var transformedPose =
                    tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.kZero));

            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                transformedPose = new Pose2d(
                        transformedPose.getTranslation(),
                        transformedPose.getRotation().plus(Rotation2d.k180deg));
            }
            return transformedPose;
        } else {
            return Pose2d.kZero;
        }
    }

    public Pose2d getIntermediatePointToDriveToForAlgaeIntaking(int tagID) {
        if (tagID >= 1 && tagID <= 22) {
            Pose2d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
            double xOffset = Units.inchesToMeters(
                    Constants.SuperstructureConstants.X_OFFSET_FROM_TAG_FOR_INTERMEDIATE_INTAKING_ALGAE_INCHES);

            Translation2d offsetFromTag = new Translation2d(xOffset, 0);

            var transformedPose =
                    tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.kZero));

            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                transformedPose = new Pose2d(
                        transformedPose.getTranslation(),
                        transformedPose.getRotation().plus(Rotation2d.k180deg));
            }
            return transformedPose;
        } else {
            return Pose2d.kZero;
        }
    }

    public Pose2d getDesiredPointToDriveToForAlgaeIntaking(int tagID) {
        if (tagID >= 1 && tagID <= 22) {
            Pose2d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
            double xOffset =
                    Units.inchesToMeters(Constants.SuperstructureConstants.X_OFFSET_FROM_TAG_FOR_INTAKING_ALGAE_INCHES);

            Translation2d offsetFromTag = new Translation2d(xOffset, 0);

            var transformedPose =
                    tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.kZero));

            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                transformedPose = new Pose2d(
                        transformedPose.getTranslation(),
                        transformedPose.getRotation().plus(Rotation2d.k180deg));
            }
            return transformedPose;
        } else {
            return Pose2d.kZero;
        }
    }

    public boolean hasCoral() {
        return intakeSubsystem.hasStraightCoral();
    }

    public boolean hasCollectedPieceInAuto() {
        return intakeSubsystem.isBackCanRangeTripped()
                && armSubsystem.getCurrentExtensionPositionInMeters() < Units.inchesToMeters(10.0);
    }

    public boolean hasHomeButtonBeenPressed() {
        return toggleInputs.isHomeButtonPressed;
    }

    public boolean hasSwitchValueChanged() {
        return pastSwitchValue != toggleInputs.switchValue;
    }

    public NeutralModeValue getNeutralMode() {
        return toggleInputs.switchValue;
    }

    public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
    }

    public Command setStateCommand(WantedSuperState superState) {
        return setStateCommand(superState, false);
    }

    public Command setStateCommand(WantedSuperState superState, boolean runIfClimberDeployed) {
        Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
        if (!runIfClimberDeployed) {
            commandToReturn = commandToReturn.onlyIf(() -> currentSuperState != CurrentSuperState.CLIMB);
        }
        return commandToReturn;
    }

    public Command configureButtonBinding(
            WantedSuperState hasStraightCoralCondition,
            WantedSuperState hasHorizontalCoralCondition,
            WantedSuperState hasAlgaeCondition,
            WantedSuperState noPieceCondition) {
        return Commands.either(
                Commands.either(
                        Commands.either(
                                setStateCommand(hasHorizontalCoralCondition),
                                setStateCommand(hasStraightCoralCondition),
                                () -> !shouldHoldStraight),
                        setStateCommand(hasAlgaeCondition),
                        intakeSubsystem::hasStraightCoral),
                setStateCommand(noPieceCondition),
                () -> intakeSubsystem.hasHorizontalCoral()
                        || intakeSubsystem.hasStraightCoral()
                        || intakeSubsystem.hasAlgae());
    }

    public void toggleReefSelectionMethod() {
        reefSelectionMethod = (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE)
                ? Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
                : Constants.SuperstructureConstants.ReefSelectionMethod.POSE;
    }

    public void toggleHasPoseBeenSetForPrematch(boolean hasPoseBeenResetPrematch) {
        this.hasPoseBeenResetPrematch = hasPoseBeenResetPrematch;
    }

    public void setIsExternalCommandAllowedToAccessLEDs(boolean isExternalCommandAllowedToAccessLEDs) {
        this.allowExternalCommandToAccessLEDS = isExternalCommandAllowedToAccessLEDs;
    }
}
