package org.frc2910.robot.subsystems.arm.extension;

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

import java.util.function.Supplier;

public class ExtensionIOTalonFX implements ExtensionIO {
    private final TalonFX extensionOne;
    private final TalonFX extensionTwo;
    private final TalonFX extensionThree;
    Follower followControlRequest;
    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final StatusSignal<Angle> extensionPositionInMeters;
    private final StatusSignal<Voltage> extensionAppliedVolts;
    private final StatusSignal<Current> extensionSupplyCurrentAmps;
    private final StatusSignal<Current> extensionStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> extensionVelocityMetersPerSec;
    private final StatusSignal<AngularAcceleration> extensionAccelerationMetersPerSecSquared;
    private final StatusSignal<Temperature> extensionOneMotorTemp;
    private final StatusSignal<Temperature> extensionTwoMotorTemp;
    private final StatusSignal<Temperature> extensionThreeMotorTemp;

    private Supplier<Rotation2d> shoulderAngleSupplier = () -> Rotation2d.kZero;

    public ExtensionIOTalonFX(PortConfiguration ports, ArmConfiguration armConfiguration) {
        extensionOne = TalonFXFactory.createDefaultTalon(ports.extensionOneID);
        extensionTwo = TalonFXFactory.createDefaultTalon(ports.extensionTwoID);
        extensionThree = TalonFXFactory.createDefaultTalon(ports.extensionThreeID);
        followControlRequest = new Follower(ports.extensionOneID.getDeviceNumber(), false);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;
        config.Slot0.kP = armConfiguration.extensionkP;
        config.Slot0.kI = armConfiguration.extensionkI;
        config.Slot0.kD = armConfiguration.extensionkD;

        config.Slot0.kS = armConfiguration.extensionkS;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotionMagic.MotionMagicAcceleration = ArmConstants.EXTENSION_ACCELERATION_CONSTRAINT;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.EXTENSION_VELOCITY_CONSTRAINT;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        extensionOne.getConfigurator().apply(config);
        extensionTwo.getConfigurator().apply(config);
        extensionThree.getConfigurator().apply(config);

        extensionTwo.setControl(followControlRequest);
        extensionThree.setControl(followControlRequest);

        extensionPositionInMeters = extensionOne.getPosition();
        extensionAppliedVolts = extensionOne.getMotorVoltage();
        extensionSupplyCurrentAmps = extensionOne.getSupplyCurrent();
        extensionStatorCurrentAmps = extensionOne.getStatorCurrent();
        extensionVelocityMetersPerSec = extensionOne.getRotorVelocity();
        extensionAccelerationMetersPerSecSquared = extensionOne.getAcceleration();
        extensionOneMotorTemp = extensionOne.getDeviceTemp();
        extensionTwoMotorTemp = extensionTwo.getDeviceTemp();
        extensionThreeMotorTemp = extensionThree.getDeviceTemp();
    }

    @Override
    public void updateInputs(ExtensionIOInputs inputs) {
        inputs.extensionPositionInMeters = (extensionPositionInMeters.getValueAsDouble()
                        - (shoulderAngleSupplier.get().getRotations() * ArmConstants.SHOULDER_EXTENSION_COUPLING_RATIO))
                * ArmConstants.EXTENSION_POSITION_COEFFICIENT;

        inputs.extensionAppliedVolts = extensionAppliedVolts.getValueAsDouble();
        inputs.extensionSupplyCurrentAmps = extensionSupplyCurrentAmps.getValueAsDouble();
        inputs.extensionStatorCurrentAmps = extensionStatorCurrentAmps.getValueAsDouble();
        inputs.extensionVelocityMetersPerSec =
                extensionVelocityMetersPerSec.getValueAsDouble() * ArmConstants.EXTENSION_POSITION_COEFFICIENT;
        inputs.extensionAccelerationMetersPerSecSquared = extensionAccelerationMetersPerSecSquared.getValueAsDouble()
                * ArmConstants.EXTENSION_POSITION_COEFFICIENT;

        inputs.extensionOneMotorTemp = extensionOneMotorTemp.getValueAsDouble();
        inputs.extensionTwoMotorTemp = extensionTwoMotorTemp.getValueAsDouble();
        inputs.extensionThreeMotorTemp = extensionThreeMotorTemp.getValueAsDouble();
    }

    @Override
    public void setTargetExtension(double positionInMeters) {
        extensionOne.setControl(positionVoltage.withPosition(
                (shoulderAngleSupplier.get().getRotations() * ArmConstants.SHOULDER_EXTENSION_COUPLING_RATIO)
                        + positionInMeters / ArmConstants.EXTENSION_POSITION_COEFFICIENT));
    }

    @Override
    public void resetExtensionPosition(double positionInMeters) {
        extensionOne.setPosition(positionInMeters / ArmConstants.EXTENSION_POSITION_COEFFICIENT);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        extensionOne.setNeutralMode(neutralMode);
        extensionTwo.setNeutralMode(neutralMode);
        extensionThree.setNeutralMode(neutralMode);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        extensionOne.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void setShoulderAngleSupplier(Supplier<Rotation2d> shoulderAngleSupplier) {
        this.shoulderAngleSupplier = shoulderAngleSupplier;
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                extensionPositionInMeters,
                extensionAppliedVolts,
                extensionSupplyCurrentAmps,
                extensionStatorCurrentAmps,
                extensionVelocityMetersPerSec,
                extensionAccelerationMetersPerSecSquared,
                extensionOneMotorTemp,
                extensionTwoMotorTemp,
                extensionThreeMotorTemp);
    }
}
