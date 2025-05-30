package org.frc2910.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import java.util.List;

public interface RobotConstants {

    SwerveDrivetrainConstants getSwerveDrivetrainConstants();

    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getModuleConstants();

    PortConfiguration getPortConfiguration();

    List<CameraConfiguration> getCameraConfigurations();

    FollowPathConfiguration getChoreoPathConfiguration();

    FollowPathConfiguration getPathPlannerPathConfiguration();

    ArmConfiguration getArmConfiguration();

    static RobotConstants getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case LOKI:
                return new Loki();
            case CYCLONE:
                return new Cyclone();
            case TYPHOON:
                return new Typhoon();
            case SIMULATION:
            default:
                // Something went wrong if this branch is reached, by default we will return our Comp Bot
                return new Spectre();
        }
    }
}
