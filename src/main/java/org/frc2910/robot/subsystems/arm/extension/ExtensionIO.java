package org.frc2910.robot.subsystems.arm.extension;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc2910.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

public interface ExtensionIO extends SubsystemDataProcessor.IODataRefresher {
    default void updateInputs(ExtensionIOInputs inputs) {}

    @AutoLog
    class ExtensionIOInputs {
        public double extensionPositionInMeters;

        public double extensionAppliedVolts;
        public double extensionSupplyCurrentAmps;
        public double extensionStatorCurrentAmps;
        public double extensionVelocityMetersPerSec;
        public double extensionAccelerationMetersPerSecSquared;

        public double extensionOneMotorTemp;
        public double extensionTwoMotorTemp;
        public double extensionThreeMotorTemp;
    }

    default void setShoulderAngleSupplier(Supplier<Rotation2d> shoulderAngleSupplier) {}

    default void setTargetExtension(double positionInMeters) {}

    default void resetExtensionPosition(double positionInMeters) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
