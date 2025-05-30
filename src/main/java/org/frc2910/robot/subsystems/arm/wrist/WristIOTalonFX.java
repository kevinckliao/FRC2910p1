package org.frc2910.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.frc2910.robot.config.ArmConfiguration;
import org.frc2910.robot.config.PortConfiguration;
import org.frc2910.robot.constants.Constants.ArmConstants;
import org.frc2910.robot.util.phoenix6.TalonFXFactory;

public class WristIOTalonFX implements WristIO {
    private TalonFX wrist;

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<Voltage> wristVoltage;
    private final StatusSignal<Current> wristSupplyCurrent;
    private final StatusSignal<Current> wristStatorCurrent;
    private final StatusSignal<Temperature> wristTemperature;
    private final StatusSignal<AngularVelocity> wristAngularVelocity;
    private final StatusSignal<AngularAcceleration> wristAngularAcceleration;

    public WristIOTalonFX(PortConfiguration ports, ArmConfiguration armConfiguration) {
        wrist = TalonFXFactory.createDefaultTalon(ports.wristID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.Slot0.kP = armConfiguration.wristkP;

        config.Slot0.kI = armConfiguration.wristkI;
        config.Slot0.kD = armConfiguration.wristkD;

        config.Slot0.kS = armConfiguration.wristkS;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.WRIST_ACCELERATION_CONSTRAINT;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.WRIST_VELOCITY_CONSTRAINT;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wrist.getConfigurator().apply(config);

        wristPosition = wrist.getRotorPosition();
        wristVoltage = wrist.getMotorVoltage();
        wristSupplyCurrent = wrist.getSupplyCurrent();
        wristStatorCurrent = wrist.getStatorCurrent();
        wristTemperature = wrist.getDeviceTemp();
        wristAngularVelocity = wrist.getRotorVelocity();
        wristAngularAcceleration = wrist.getAcceleration();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAngle =
                Rotation2d.fromRadians(wristPosition.getValueAsDouble() * ArmConstants.WRIST_POSITION_COEFFICIENT);

        inputs.wristAppliedVolts = wristVoltage.getValueAsDouble();
        inputs.wristSupplyCurrentAmps = wristSupplyCurrent.getValueAsDouble();
        inputs.wristStatorCurrentAmps = wristStatorCurrent.getValueAsDouble();
        inputs.wristMotorTemp = wristTemperature.getValueAsDouble();

        inputs.wristAngularVelocityRadPerSec =
                wristAngularVelocity.getValueAsDouble() * ArmConstants.WRIST_POSITION_COEFFICIENT;

        inputs.wristAngularAccelerationRadPerSecSquared =
                wristAngularAcceleration.getValueAsDouble() * ArmConstants.WRIST_POSITION_COEFFICIENT;
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        wrist.setControl(positionVoltage.withPosition(target.getRadians() / ArmConstants.WRIST_POSITION_COEFFICIENT));
    }

    @Override
    public void resetWristAngle(Rotation2d angle) {
        wrist.setPosition(angle.getRadians() / ArmConstants.WRIST_POSITION_COEFFICIENT);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        wrist.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        wrist.setNeutralMode(neutralMode);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                wristPosition,
                wristVoltage,
                wristSupplyCurrent,
                wristStatorCurrent,
                wristTemperature,
                wristAngularVelocity,
                wristAngularAcceleration);
    }
}
