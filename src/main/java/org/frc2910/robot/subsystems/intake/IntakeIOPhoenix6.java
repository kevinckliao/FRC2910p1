package org.frc2910.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import org.frc2910.robot.config.PortConfiguration;
import org.frc2910.robot.util.phoenix6.TalonFXFactory;

public class IntakeIOPhoenix6 implements IntakeIO {
    private final TalonFX topMotor;
    private final TalonFX leftVerticalMotor;
    private final TalonFX rightVerticalMotor;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange backCANRange;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange frontCANRange;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange frontLeftCANRange;

    @SuppressWarnings("FieldCanBeLocal")
    private final CANrange frontRightCANRange;

    private final VoltageOut topRequest = new VoltageOut(0.0);
    private final VoltageOut leftVerticalRequest = new VoltageOut(0.0);
    private final VoltageOut rightVerticalRequest = new VoltageOut(0.0);

    private final StatusSignal<Voltage> topVoltage;
    private final StatusSignal<Current> topSupplyCurrent;
    private final StatusSignal<Current> topStatorCurrent;
    private final StatusSignal<Temperature> topTemperature;
    private final StatusSignal<AngularVelocity> topVelocityRPS;

    private final StatusSignal<Voltage> leftVerticalVoltage;
    private final StatusSignal<Current> leftVerticalSupplyCurrent;
    private final StatusSignal<Current> leftVerticalStatorCurrent;
    private final StatusSignal<Temperature> leftVerticalTemperature;
    private final StatusSignal<AngularVelocity> leftVerticalVelocityRPS;

    private final StatusSignal<Voltage> rightVerticalVoltage;
    private final StatusSignal<Current> rightVerticalSupplyCurrent;
    private final StatusSignal<Current> rightVerticalStatorCurrent;
    private final StatusSignal<Temperature> rightVerticalTemperature;
    private final StatusSignal<AngularVelocity> rightVerticalVelocityRPS;

    private final StatusSignal<Double> backSignalStrength;
    private final StatusSignal<Boolean> isBackCANTRangeTripped;
    private final StatusSignal<Distance> backCANRangeDistance;

    private final StatusSignal<Double> frontSignalStrength;
    private final StatusSignal<Boolean> isFrontCANRangeTripped;
    private final StatusSignal<Distance> frontCANRangeDistance;

    private final StatusSignal<Double> frontRightSignalStrength;
    private final StatusSignal<Boolean> isFrontRightCANRangeTripped;
    private final StatusSignal<Distance> frontRightCANRangeDistance;

    private final StatusSignal<Double> frontLeftSignalStrength;
    private final StatusSignal<Boolean> isFrontLeftCANRangeTripped;
    private final StatusSignal<Distance> frontLeftCANRangeDistance;

    public IntakeIOPhoenix6(PortConfiguration ports) {
        topMotor = TalonFXFactory.createDefaultTalon(ports.intakeTopMotorID);
        rightVerticalMotor = TalonFXFactory.createDefaultTalon(ports.intakeRightVerticalMotorID);
        leftVerticalMotor = TalonFXFactory.createDefaultTalon(ports.intakeLeftVerticalMotorID);

        var talonFxConfiguration = new TalonFXConfiguration();
        talonFxConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFxConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFxConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFxConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFxConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0;
        talonFxConfiguration.CurrentLimits.StatorCurrentLimit = 100.0;

        rightVerticalMotor.getConfigurator().apply(talonFxConfiguration);
        topMotor.getConfigurator().apply(talonFxConfiguration);

        talonFxConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftVerticalMotor.getConfigurator().apply(talonFxConfiguration);

        var canRangeConfigs = new CANrangeConfiguration();
        canRangeConfigs.ProximityParams.ProximityHysteresis = 0.01;
        canRangeConfigs.ProximityParams.ProximityThreshold = 0.07;
        canRangeConfigs.ProximityParams.MinSignalStrengthForValidMeasurement = 8000;
        backCANRange = new CANrange(ports.intakeBackCANRangeID.getDeviceNumber(), ports.intakeBackCANRangeID.getBus());
        frontCANRange =
                new CANrange(ports.intakeFrontCANRangeID.getDeviceNumber(), ports.intakeFrontCANRangeID.getBus());
        frontLeftCANRange = new CANrange(
                ports.intakeFrontLeftCANRangeID.getDeviceNumber(), ports.intakeFrontLeftCANRangeID.getBus());
        frontRightCANRange = new CANrange(
                ports.intakeFrontRightCANRangeID.getDeviceNumber(), ports.intakeFrontRightCANRangeID.getBus());

        backCANRange.getConfigurator().apply(canRangeConfigs);
        frontCANRange.getConfigurator().apply(canRangeConfigs);
        frontLeftCANRange.getConfigurator().apply(canRangeConfigs);
        frontRightCANRange.getConfigurator().apply(canRangeConfigs);

        topVoltage = topMotor.getMotorVoltage();
        topSupplyCurrent = topMotor.getSupplyCurrent();
        topStatorCurrent = topMotor.getStatorCurrent();
        topTemperature = topMotor.getDeviceTemp();
        topVelocityRPS = topMotor.getRotorVelocity();

        leftVerticalVoltage = leftVerticalMotor.getMotorVoltage();
        leftVerticalSupplyCurrent = leftVerticalMotor.getSupplyCurrent();
        leftVerticalStatorCurrent = leftVerticalMotor.getStatorCurrent();
        leftVerticalTemperature = leftVerticalMotor.getDeviceTemp();
        leftVerticalVelocityRPS = leftVerticalMotor.getRotorVelocity();

        rightVerticalVoltage = rightVerticalMotor.getMotorVoltage();
        rightVerticalSupplyCurrent = rightVerticalMotor.getSupplyCurrent();
        rightVerticalStatorCurrent = rightVerticalMotor.getStatorCurrent();
        rightVerticalTemperature = rightVerticalMotor.getDeviceTemp();
        rightVerticalVelocityRPS = rightVerticalMotor.getRotorVelocity();

        isBackCANTRangeTripped = backCANRange.getIsDetected();
        backSignalStrength = backCANRange.getSignalStrength();
        backCANRangeDistance = backCANRange.getDistance();

        isFrontCANRangeTripped = frontCANRange.getIsDetected();
        frontSignalStrength = frontCANRange.getSignalStrength();
        frontCANRangeDistance = frontCANRange.getDistance();

        isFrontLeftCANRangeTripped = frontLeftCANRange.getIsDetected();
        frontLeftSignalStrength = frontLeftCANRange.getSignalStrength();
        frontLeftCANRangeDistance = frontLeftCANRange.getDistance();

        isFrontRightCANRangeTripped = frontRightCANRange.getIsDetected();
        frontRightSignalStrength = frontRightCANRange.getSignalStrength();
        frontRightCANRangeDistance = frontRightCANRange.getDistance();
    }

    private VoltageOut voltageControl(VoltageOut controller, double voltage) {
        return controller.withOutput(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    @Override
    public void setTopMotorVoltage(double voltage) {
        topMotor.setControl(voltageControl(topRequest, voltage));
    }

    @Override
    public void setLeftVerticalMotorVoltage(double voltage) {
        leftVerticalMotor.setControl(voltageControl(leftVerticalRequest, voltage));
    }

    @Override
    public void setRightVerticalMotorVoltage(double voltage) {
        rightVerticalMotor.setControl(voltageControl(rightVerticalRequest, voltage));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.topVoltage = topVoltage.getValueAsDouble();
        inputs.topSupplyCurrent = topSupplyCurrent.getValueAsDouble();
        inputs.topStatorCurrent = topStatorCurrent.getValueAsDouble();
        inputs.topTemperature = topTemperature.getValueAsDouble();
        inputs.topVelocityRPS = topVelocityRPS.getValueAsDouble();

        inputs.rightVerticalVoltage = rightVerticalVoltage.getValueAsDouble();
        inputs.rightVerticalSupplyCurrent = rightVerticalSupplyCurrent.getValueAsDouble();
        inputs.rightVerticalStatorCurrent = rightVerticalStatorCurrent.getValueAsDouble();
        inputs.rightVerticalTemperature = rightVerticalTemperature.getValueAsDouble();
        inputs.rightVerticalRPS = rightVerticalVelocityRPS.getValueAsDouble();

        inputs.leftVerticalVoltage = leftVerticalVoltage.getValueAsDouble();
        inputs.leftVerticalSupplyCurrent = leftVerticalSupplyCurrent.getValueAsDouble();
        inputs.leftVerticalStatorCurrent = leftVerticalStatorCurrent.getValueAsDouble();
        inputs.leftVerticalTemperature = leftVerticalTemperature.getValueAsDouble();
        inputs.leftVerticalRPS = leftVerticalVelocityRPS.getValueAsDouble();

        inputs.isBackCANTRangeTripped = isBackCANTRangeTripped.getValue();
        inputs.backCANRangeSignalStrength = backSignalStrength.getValueAsDouble();
        inputs.backCANRangeDistanceInMeters = backCANRangeDistance.getValueAsDouble();

        inputs.isFrontCANRangeTripped = isFrontCANRangeTripped.getValue();
        inputs.frontCANRangeSignalStrength = frontSignalStrength.getValueAsDouble();
        inputs.frontCANRangeDistanceInMeters = frontCANRangeDistance.getValueAsDouble();

        inputs.isFrontRightCANRangeTripped = isFrontRightCANRangeTripped.getValue();
        inputs.frontRightCANRangeSignalStrength = frontRightSignalStrength.getValueAsDouble();
        inputs.frontRightCANRangeDistanceInMeters = frontRightCANRangeDistance.getValueAsDouble();

        inputs.isFrontLeftCANRangeTripped = isFrontLeftCANRangeTripped.getValue();
        inputs.frontLeftCANRangeSignalStrength = frontLeftSignalStrength.getValueAsDouble();
        inputs.frontLeftCANRangeDistanceInMeters = frontLeftCANRangeDistance.getValueAsDouble();
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                topVoltage,
                topSupplyCurrent,
                topStatorCurrent,
                topTemperature,
                topVelocityRPS,
                rightVerticalVoltage,
                rightVerticalSupplyCurrent,
                rightVerticalStatorCurrent,
                rightVerticalTemperature,
                rightVerticalVelocityRPS,
                leftVerticalVoltage,
                leftVerticalSupplyCurrent,
                leftVerticalStatorCurrent,
                leftVerticalTemperature,
                leftVerticalVelocityRPS,
                isBackCANTRangeTripped,
                backSignalStrength,
                backCANRangeDistance,
                frontSignalStrength,
                isFrontCANRangeTripped,
                frontCANRangeDistance,
                frontRightSignalStrength,
                isFrontRightCANRangeTripped,
                frontRightCANRangeDistance,
                frontLeftSignalStrength,
                isFrontLeftCANRangeTripped,
                frontLeftCANRangeDistance);
    }
}
