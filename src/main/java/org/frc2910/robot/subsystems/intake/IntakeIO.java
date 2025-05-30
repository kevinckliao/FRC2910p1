package org.frc2910.robot.subsystems.intake;

import org.frc2910.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends SubsystemDataProcessor.IODataRefresher {
    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public double topVoltage = 0.0;
        public double leftVerticalVoltage = 0.0;
        public double rightVerticalVoltage = 0.0;

        public double topSupplyCurrent = 0.0;
        public double topStatorCurrent = 0.0;
        public double leftVerticalSupplyCurrent = 0.0;
        public double leftVerticalStatorCurrent = 0.0;
        public double rightVerticalSupplyCurrent = 0.0;
        public double rightVerticalStatorCurrent = 0.0;

        public double topTemperature = 0.0;
        public double leftVerticalTemperature = 0.0;
        public double rightVerticalTemperature = 0.0;

        public double topVelocityRPS = 0.0;
        public double leftVerticalRPS = 0.0;
        public double rightVerticalRPS = 0.0;

        public boolean isBackCANTRangeTripped = false;
        public boolean isFrontCANRangeTripped = false;
        public boolean isFrontRightCANRangeTripped = false;
        public boolean isFrontLeftCANRangeTripped = false;

        public double backCANRangeSignalStrength = 0.0;
        public double frontCANRangeSignalStrength = 0.0;
        public double frontRightCANRangeSignalStrength = 0.0;
        public double frontLeftCANRangeSignalStrength = 0.0;

        public double backCANRangeDistanceInMeters = 0.0;
        public double frontCANRangeDistanceInMeters = 0.0;
        public double frontRightCANRangeDistanceInMeters = 0.0;
        public double frontLeftCANRangeDistanceInMeters = 0.0;
    }

    default void setTopMotorVoltage(double voltage) {}

    default void setLeftVerticalMotorVoltage(double voltage) {}

    default void setRightVerticalMotorVoltage(double voltage) {}

    @Override
    default void refreshData() {}
}
