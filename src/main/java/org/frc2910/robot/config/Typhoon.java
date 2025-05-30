package org.frc2910.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import org.frc2910.robot.util.CanDeviceId;

import java.util.List;

public class Typhoon implements RobotConstants {
    private static final String CANIVORE_CANBUS_NAME = "CANivore";

    // Ports and IDs
    private static final CanDeviceId GYRO = new CanDeviceId(2, CANIVORE_CANBUS_NAME);

    // Swerve Modules
    private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(1, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(2, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(18, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(17, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(2, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(7, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(8, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(3, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(11, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(12, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(4, CANIVORE_CANBUS_NAME);

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.92); // Old value 1.95

    /**
     * Ratio between the drive motor shaft and the output shaft the wheel is mounted on.
     */
    private static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

    /**
     * Ratio between the steer motor shaft and the steer output shaft.
     */
    private static final double MK4i_STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final double MK4n_STEER_GEAR_RATIO = 18.75;

    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = 0.0;
    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(20.75);

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = Units.inchesToMeters(20.75);

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(16.0);

    /**
     * The maximum angular speed of the robot in radians per second.
     * If set to 0, the value is calculated using the max speed in meters per second
     * and the wheelbase radius.
     */
    private static final double MAX_SPEED_IN_RADIANS_PER_SECOND_LIMIT = Units.degreesToRadians(360);

    // CANcoder offsets of the swerve modules
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = -0.458;
    // -Rotation2d.fromRadians(1.342).getRotations();
    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = -0.454;
    // -Rotation2d.fromRadians(-1.519).getRotations();
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = 0.039;
    // -Rotation2d.fromRadians(-2.586).getRotations();
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = 0.27;
    // -Rotation2d.fromRadians(0.550).getRotations();

    private static final int GYRO_MOUNTING_ANGLE = -90;
    private static final double GYRO_ERROR = 0.28;

    // ==================================================================================
    // Path-following constants
    // ==================================================================================
    private static final double FOLLOW_PATH_TRANSLATION_KP = 5.0; // used to be 5.0
    private static final double FOLLOW_PATH_TRANSLATION_KI = 0.0;
    private static final double FOLLOW_PATH_TRANSLATION_KD = 0.0;
    private static final double FOLLOW_PATH_ROTATION_KP = 8.0;
    private static final double FOLLOW_PATH_ROTATION_KI = 0.0;
    private static final double FOLLOW_PATH_ROTATION_KD = 0.0;

    // Robot configuration
    private final PortConfiguration portConfiguration;
    private final SwerveDrivetrainConstants swerveDrivetrainConstants;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            moduleConstants;
    private final Pigeon2Constants pigeon2Constants;

    private final CameraConfiguration limelightConfiguration;

    private final FollowPathConfiguration followPathConfiguration;

    private final ArmConfiguration armConfiguration;

    public Typhoon() {
        // Initialize PortConfiguration
        portConfiguration = new PortConfiguration().withCANBus(CANIVORE_CANBUS_NAME);

        Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseYaw = GYRO_MOUNTING_ANGLE;
        pigeon2Configuration.GyroTrim.GyroScalarY = GYRO_ERROR;

        swerveDrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName("CANivore")
                .withPigeon2Id(GYRO.getDeviceNumber())
                .withPigeon2Configs(pigeon2Configuration);

        moduleConstants = new SwerveModuleConstants[4];
        moduleConstants[0] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[1] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_RIGHT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[2] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(BACK_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(BACK_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4i_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[3] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(BACK_RIGHT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(BACK_RIGHT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4i_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        pigeon2Constants = new Pigeon2Constants()
                .withCanDeviceId(GYRO)
                .withMountPoseYaw(GYRO_MOUNTING_ANGLE)
                .withGyroScalarZ(GYRO_ERROR);

        // TODO: Configure with 2024 Limelight Position Measurements
        limelightConfiguration = new CameraConfiguration()
                .withHeightOffset(Units.inchesToMeters(8.5))
                .withMountingPitch(Units.degreesToRadians(26.79));

        followPathConfiguration = new FollowPathConfiguration()
                .withTranslationKp(FOLLOW_PATH_TRANSLATION_KP)
                .withTranslationKi(FOLLOW_PATH_TRANSLATION_KI)
                .withTranslationKd(FOLLOW_PATH_TRANSLATION_KD)
                .withRotationKp(FOLLOW_PATH_ROTATION_KP)
                .withRotationKi(FOLLOW_PATH_ROTATION_KI)
                .withRotationKd(FOLLOW_PATH_ROTATION_KD);

        armConfiguration = new ArmConfiguration();
    }

    @Override
    public SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
        return swerveDrivetrainConstants;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            getModuleConstants() {
        return moduleConstants;
    }

    @Override
    public PortConfiguration getPortConfiguration() {
        return portConfiguration;
    }

    @Override
    public List<CameraConfiguration> getCameraConfigurations() {
        return List.of(limelightConfiguration);
    }

    public FollowPathConfiguration getChoreoPathConfiguration() {
        return followPathConfiguration;
    }

    @Override
    public FollowPathConfiguration getPathPlannerPathConfiguration() {
        return followPathConfiguration;
    }

    @Override
    public ArmConfiguration getArmConfiguration() {
        return armConfiguration;
    }
}
