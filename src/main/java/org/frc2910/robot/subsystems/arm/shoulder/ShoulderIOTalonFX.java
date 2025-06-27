package org.frc2910.robot.subsystems.arm.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
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

public class ShoulderIOTalonFX implements ShoulderIO {
    public final TalonFX shoulderOne;
    public final TalonFX shoulderTwo;
    public final TalonFX shoulderThree;

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

    private final StatusSignal<Angle> shoulderAngle;
    private final StatusSignal<Voltage> shoulderAppliedVolts;
    private final StatusSignal<Current> shoulderSupplyCurrentAmps;
    private final StatusSignal<Current> shoulderStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> shoulderAngularVelocityRadPerSec;
    private final StatusSignal<AngularAcceleration> shoulderAngularAccelerationRadPerSecSquared;
    private final StatusSignal<Temperature> shoulderMotorOneTemp;
    private final StatusSignal<Temperature> shoulderMotorTwoTemp;
    private final StatusSignal<Temperature> shoulderMotorThreeTemp;

    public ShoulderIOTalonFX(PortConfiguration ports, ArmConfiguration armConfiguration) {
        Follower followerWithoutInverse = new Follower(ports.shoulderOneID.getDeviceNumber(), false);
        shoulderOne = TalonFXFactory.createDefaultTalon(ports.shoulderOneID);
        shoulderTwo = TalonFXFactory.createDefaultTalon(ports.shoulderTwoID);
        shoulderThree = TalonFXFactory.createDefaultTalon(ports.shoulderThreeID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimit = 90.0;

        config.Slot0.kP = armConfiguration.shoulderkP;
        config.Slot0.kI = armConfiguration.shoulderkI;
        config.Slot0.kD = armConfiguration.shoulderkD;

        config.Slot0.kS = armConfiguration.shoulderkS;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.SHOULDER_ACCELERATION_CONSTRAINT;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.SHOULDER_VELOCITY_CONSTRAINT;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shoulderOne.getConfigurator().apply(config);
        shoulderTwo.getConfigurator().apply(config);
        shoulderThree.getConfigurator().apply(config);

        shoulderTwo.setControl(followerWithoutInverse);
        shoulderThree.setControl(followerWithoutInverse);

        shoulderAngle = shoulderOne.getPosition();
        shoulderAppliedVolts = shoulderOne.getMotorVoltage();
        shoulderSupplyCurrentAmps = shoulderOne.getSupplyCurrent();
        shoulderStatorCurrentAmps = shoulderOne.getStatorCurrent();
        shoulderAngularVelocityRadPerSec = shoulderOne.getRotorVelocity();
        shoulderAngularAccelerationRadPerSecSquared = shoulderOne.getAcceleration();
        shoulderMotorOneTemp = shoulderOne.getDeviceTemp();
        shoulderMotorTwoTemp = shoulderTwo.getDeviceTemp();
        shoulderMotorThreeTemp = shoulderThree.getDeviceTemp();
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        inputs.shoulderAngle =
                Rotation2d.fromRadians(shoulderAngle.getValueAsDouble() * ArmConstants.SHOULDER_POSITION_COEFFICIENT);

        inputs.shoulderAppliedVolts = shoulderAppliedVolts.getValueAsDouble();

        inputs.shoulderSupplyCurrentAmps = shoulderSupplyCurrentAmps.getValueAsDouble();
        inputs.shoulderStatorCurrentAmps = shoulderStatorCurrentAmps.getValueAsDouble();
        inputs.shoulderAngularVelocityRadPerSec =
                shoulderAngularVelocityRadPerSec.getValueAsDouble() * ArmConstants.SHOULDER_POSITION_COEFFICIENT;
        inputs.shoulderAngularAccelerationRadPerSecSquared =
                shoulderAngularAccelerationRadPerSecSquared.getValueAsDouble()
                        * ArmConstants.SHOULDER_POSITION_COEFFICIENT;

        inputs.shoulderOneMotorTemp = shoulderMotorOneTemp.getValueAsDouble();
        inputs.shoulderTwoMotorTemp = shoulderMotorTwoTemp.getValueAsDouble();
        inputs.shoulderThreeMotorTemp = shoulderMotorThreeTemp.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        shoulderOne.setControl(
                motionMagicVoltage.withPosition(target.getRadians() / ArmConstants.SHOULDER_POSITION_COEFFICIENT));
    }

    @Override
    public void resetShoulderAngle(Rotation2d angle) {
        shoulderOne.setPosition(angle.getRadians() / ArmConstants.SHOULDER_POSITION_COEFFICIENT);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        shoulderOne.setControl(dutyCycleOut.withOutput(dutyCycle));
        // Only main motor needs to be controlled, others will follow
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        shoulderOne.setNeutralMode(neutralMode);
        shoulderTwo.setNeutralMode(neutralMode);
        shoulderThree.setNeutralMode(neutralMode);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                shoulderAngle,
                shoulderAppliedVolts,
                shoulderSupplyCurrentAmps,
                shoulderStatorCurrentAmps,
                shoulderAngularVelocityRadPerSec,
                shoulderAngularAccelerationRadPerSecSquared,
                shoulderMotorOneTemp,
                shoulderMotorTwoTemp,
                shoulderMotorThreeTemp);
    }
}
