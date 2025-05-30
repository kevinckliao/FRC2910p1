package org.frc2910.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc2910.robot.constants.Constants;
import org.frc2910.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.Logger;

import static org.frc2910.robot.constants.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double climberZeroTimestamp = Double.NaN;
    private boolean isClimberHomed = false;

    public enum WantedState {
        IDLE,
        STOWED,
        INTAKE,
        REJECT,
        MOVE_TO_POSITION,
        INTAKE_AND_MOVE,
        HOME,
        HOLD
    }

    private enum SystemState {
        IDLED,
        STOWED,
        INTAKING,
        REJECTING,
        MOVING_TO_POSITION,
        INTAKE_AND_MOVE,
        HOMING,
        HOLDING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;
    private WantedState previousWantedState = WantedState.IDLE;

    private boolean hasCageEntered = false;
    private boolean isAllowedToCheckIfCageHasEntered = false;

    private double carriageSetpointInMeters = 0.0;
    private double climberRollerVoltageSetpoint = 0.0;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
                () -> {
                    synchronized (inputs) {
                        io.updateInputs(inputs);
                    }
                },
                io);
    }

    @Override
    public void periodic() {
        synchronized (inputs) {
            Logger.processInputs("Subsystems/Climber", inputs);

            SystemState newState = handleStateTransition();

            if (newState != systemState) {
                Logger.recordOutput("Subsystems/Climber/SystemState", newState.toString());
                systemState = newState;
            }
            Logger.recordOutput("Subsystems/Climber/WantedState", wantedState);
            Logger.recordOutput("Subsystems/Climber/hasCage", getHasCage());
            Logger.recordOutput("Subsystems/Climber/isCarriageAtPosition", isCarriageAtPosition(0.0));

            if (DriverStation.isDisabled()) {
                systemState = SystemState.IDLED;
            }

            switch (systemState) {
                case INTAKING:
                    io.setClimberIntakeVoltage(Constants.ClimberConstants.INTAKE_MOTOR_VOLTAGE);
                    io.setCarriageTargetExtensionInMeters(carriageSetpointInMeters);
                    break;
                case REJECTING:
                    io.setClimberIntakeVoltage(Constants.ClimberConstants.REJECT_MOTOR_VOLTAGE);
                    break;
                case STOWED:
                    isAllowedToCheckIfCageHasEntered = false;
                    hasCageEntered = false;
                    io.setClimberIntakeVoltage(0.0);
                    io.setCarriageTargetExtensionInMeters(0.0);
                    break;
                case INTAKE_AND_MOVE:
                    io.setClimberIntakeVoltage(climberRollerVoltageSetpoint);
                    io.setCarriageTargetExtensionInMeters(carriageSetpointInMeters);
                    break;
                case MOVING_TO_POSITION:
                    isAllowedToCheckIfCageHasEntered = false;
                    io.setClimberIntakeVoltage(0.0);
                    io.setCarriageTargetExtensionInMeters(carriageSetpointInMeters);
                    break;
                case HOMING:
                    isAllowedToCheckIfCageHasEntered = false;
                    hasCageEntered = false;
                    io.setClimberIntakeVoltage(0.0);
                    io.setDutyCycleOutputForCarriage(Constants.ClimberConstants.CLIMBER_DUTY_CYCLE_FOR_ZEROING);
                    break;
                case HOLDING:
                    isAllowedToCheckIfCageHasEntered = false;
                    io.setClimberIntakeVoltage(0.0);
                    io.setCarriageTargetExtensionInMeters(carriageSetpointInMeters);
                    break;
                default:
                    io.setClimberIntakeVoltage(0.0);
                    io.setDutyCycleOutputForCarriage(0.0);
                    break;
            }

            previousWantedState = wantedState;
        }
    }

    private SystemState handleStateTransition() {
        if (!isClimberHomed && wantedState != WantedState.HOME) {
            return SystemState.IDLED;
        }
        switch (wantedState) {
            case INTAKE -> {
                if (hasCageEntered) {
                    return SystemState.HOLDING;
                } else if (!isAllowedToCheckIfCageHasEntered) {
                    isAllowedToCheckIfCageHasEntered =
                            inputs.carriageSupplyCurrent <= CLIMBER_CARRIAGE_CAGE_SUPPLY_CURRENT_THRESHOLD
                                    && inputs.climberVelocityRPS >= 90.0;
                } else {
                    hasCageEntered = inputs.climberVelocityRPS <= CLIMBER_CARRIAGE_CAGE_VELOCITY_RPS_THRESHOLD;
                    if (hasCageEntered) {
                        return SystemState.HOLDING;
                    }
                }
                return SystemState.INTAKING;
            }
            case REJECT -> {
                return SystemState.REJECTING;
            }
            case STOWED -> {
                return SystemState.STOWED;
            }
            case MOVE_TO_POSITION -> {
                return SystemState.MOVING_TO_POSITION;
            }
            case HOME -> {
                if (previousWantedState != WantedState.HOME) {
                    isClimberHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (Math.abs(inputs.carriageVelocityMetersPerSecond)
                            <= Constants.ClimberConstants.CLIMBER_ZERO_VELOCITY_THRESHOLD_METERS_PER_SECOND) {
                        if (Double.isNaN(climberZeroTimestamp)) {
                            climberZeroTimestamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING;
                        } else if ((Timer.getFPGATimestamp() - climberZeroTimestamp)
                                >= Constants.ClimberConstants.CLIMBER_ZERO_VELOCITY_TIME_PERIOD) {
                            climberZeroTimestamp = Double.NaN;
                            isClimberHomed = true;
                            io.resetCarriagePositionInMeters(CLIMBER_DRIVEN_HOME_RESET_POSITION_METERS);
                            setWantedState(WantedState.IDLE);
                            return SystemState.IDLED;
                        }
                    } else {
                        return SystemState.HOMING;
                    }
                }
            }
            case INTAKE_AND_MOVE -> {
                return SystemState.INTAKE_AND_MOVE;
            }
            case HOLD -> {
                return SystemState.HOLDING;
            }
            default -> {
                return SystemState.IDLED;
            }
        }
        return SystemState.IDLED;
    }

    public void tareCarriage() {
        isClimberHomed = true;
        synchronized (inputs) {
            io.resetCarriagePositionInMeters(CLIMBER_BUTTON_HOME_RESET_POSITION_METERS);
        }
    }

    public boolean isCarriageAtPosition(double positionInMeters) {
        synchronized (inputs) {
            return MathUtil.isNear(
                    positionInMeters,
                    inputs.carriagePositionInMeters,
                    Constants.ClimberConstants.CLIMBER_CARRIAGE_SETPOINT_TOLERANCE);
        }
    }

    public void setCarriageNeutralMode(NeutralModeValue neutralMode) {
        synchronized (inputs) {
            io.setNeutralMode(neutralMode);
        }
    }

    public void setDesiredCarriagePositionAndRollerVoltage(double position, double voltage) {
        setWantedState(WantedState.INTAKE_AND_MOVE);
        this.carriageSetpointInMeters = position;
        this.climberRollerVoltageSetpoint = voltage;
    }

    public void setDesiredCarriagePosition(double position) {
        setWantedState(WantedState.MOVE_TO_POSITION);
        this.carriageSetpointInMeters = position;
    }

    public boolean hasHomeCompleted() {
        return isClimberHomed;
    }

    public boolean getHasCage() {
        return hasCageEntered;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setIntakeAndMoveState(double intakeSetpoint, double carriageSetpoint) {
        setWantedState(WantedState.INTAKE_AND_MOVE);
        this.climberRollerVoltageSetpoint = intakeSetpoint;
        this.carriageSetpointInMeters = carriageSetpoint;
    }
}
