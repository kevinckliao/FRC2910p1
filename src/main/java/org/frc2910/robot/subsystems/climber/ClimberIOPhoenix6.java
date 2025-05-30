package org.frc2910.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.frc2910.robot.config.PortConfiguration;
import org.frc2910.robot.constants.Constants.ClimberConstants;
import org.frc2910.robot.util.phoenix6.TalonFXFactory;

public class ClimberIOPhoenix6 implements ClimberIO {
    private final TalonFX climberRollerMotor;
    private final TalonFX carriageMotor;

    private final VoltageOut voltageController = new VoltageOut(0.0);
    private final MotionMagicVoltage carriageController = new MotionMagicVoltage(0.0);

    private final StatusSignal<Voltage> climberRollerVoltage;
    private final StatusSignal<Current> climberRollerSupplyCurrent;
    private final StatusSignal<Current> climberRollerStatorCurrent;
    private final StatusSignal<Temperature> climberRollerTemperature;
    private final StatusSignal<AngularVelocity> climberRollerVelocity;

    private final StatusSignal<Angle> carriagePositionInMeters;
    private final StatusSignal<AngularVelocity> carriageVelocity;
    private final StatusSignal<Current> carriageSupplyCurrent;
    private final StatusSignal<Current> carriageStatorCurrent;
    private final StatusSignal<Temperature> carriageTemperature;

    public ClimberIOPhoenix6(PortConfiguration ports) {
        climberRollerMotor = TalonFXFactory.createDefaultTalon(ports.climberIntakeID);
        carriageMotor = TalonFXFactory.createDefaultTalon(ports.carriageID);

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        configuration.CurrentLimits.SupplyCurrentLimit = 30.0;
        configuration.CurrentLimits.StatorCurrentLimit = 50.0;

        configuration.Slot0.kP = 5.0;
        configuration.Slot0.kI = 0;
        configuration.Slot0.kD = 0;

        climberRollerMotor.getConfigurator().apply(configuration);

        configuration.MotionMagic.MotionMagicAcceleration = ClimberConstants.CLIMBER_ACCELERATION_CONSTRAINT;
        configuration.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.CLIMBER_VELOCITY_CONSTRAINT;

        carriageMotor.getConfigurator().apply(configuration);

        climberRollerVoltage = climberRollerMotor.getMotorVoltage();
        climberRollerSupplyCurrent = climberRollerMotor.getSupplyCurrent();
        climberRollerStatorCurrent = climberRollerMotor.getStatorCurrent();
        climberRollerTemperature = climberRollerMotor.getDeviceTemp();
        carriagePositionInMeters = carriageMotor.getPosition();
        carriageVelocity = carriageMotor.getRotorVelocity();
        carriageSupplyCurrent = carriageMotor.getSupplyCurrent();
        carriageStatorCurrent = carriageMotor.getStatorCurrent();
        carriageTemperature = carriageMotor.getDeviceTemp();
        climberRollerVelocity = climberRollerMotor.getVelocity();
    }

    @Override
    public void setClimberIntakeVoltage(double voltage) {
        climberRollerMotor.setControl(voltageController.withOutput(voltage));
    }

    @Override
    public void resetCarriagePositionInMeters(double position) {
        carriageMotor.setPosition(position / ClimberConstants.CLIMBER_POSITION_COEFFICIENT);
    }

    @Override
    public void setCarriageTargetExtensionInMeters(double desiredExtension) {
        carriageMotor.setControl(
                carriageController.withPosition(desiredExtension / ClimberConstants.CLIMBER_POSITION_COEFFICIENT));
    }

    @Override
    public void setDutyCycleOutputForCarriage(double dutyCycleOutput) {
        carriageMotor.set(dutyCycleOutput);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        carriageMotor.setNeutralMode(neutralMode);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Update roller motor inputs
        inputs.climberRollerVoltage = climberRollerVoltage.getValueAsDouble();
        inputs.climberRollerSupplyCurrent = climberRollerSupplyCurrent.getValueAsDouble();
        inputs.climberRollerStatorCurrent = climberRollerStatorCurrent.getValueAsDouble();
        inputs.climberRollerTemperature = climberRollerTemperature.getValueAsDouble();
        inputs.climberVelocityRPS = climberRollerVelocity.getValueAsDouble();

        inputs.carriagePositionInMeters =
                carriagePositionInMeters.getValueAsDouble() * ClimberConstants.CLIMBER_POSITION_COEFFICIENT;
        inputs.carriageVelocityMetersPerSecond =
                carriageVelocity.getValueAsDouble() * ClimberConstants.CLIMBER_POSITION_COEFFICIENT;
        inputs.carriageSupplyCurrent = carriageSupplyCurrent.getValueAsDouble();
        inputs.carriageStatorCurrent = carriageStatorCurrent.getValueAsDouble();
        inputs.carriageTemperature = carriageTemperature.getValueAsDouble();
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                climberRollerVoltage,
                climberRollerSupplyCurrent,
                climberRollerStatorCurrent,
                climberRollerTemperature,
                carriagePositionInMeters,
                carriageVelocity,
                carriageSupplyCurrent,
                carriageStatorCurrent,
                carriageTemperature,
                climberRollerVelocity);
    }
}
