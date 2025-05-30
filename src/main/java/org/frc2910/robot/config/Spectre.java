package org.frc2910.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import org.frc2910.robot.util.CanDeviceId;

import java.util.List;

public class Spectre implements RobotConstants {

    private static final String CANIVORE_CANBUS_NAME = "CANivore";

    // CANdle
    private static final CanDeviceId CANDLE_ID = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    // Ports and IDs
    private static final CanDeviceId GYRO = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    // Swerve Modules
    private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(1, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(2, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(1, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(3, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(4, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(2, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(5, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(6, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(3, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(7, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(8, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(4, CANIVORE_CANBUS_NAME);

    // Arm
    private static final CanDeviceId EXTENSION_ONE_MOTOR = new CanDeviceId(9, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId EXTENSION_TWO_MOTOR = new CanDeviceId(10, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId EXTENSION_THREE_MOTOR = new CanDeviceId(11, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId SHOULDER_ONE_MOTOR = new CanDeviceId(12, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId SHOULDER_TWO_MOTOR = new CanDeviceId(13, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId SHOULDER_THREE_MOTOR = new CanDeviceId(14, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId WRIST_MOTOR = new CanDeviceId(15, CANIVORE_CANBUS_NAME);

    // Climber
    private static final CanDeviceId CLIMBER_CARRIAGE = new CanDeviceId(16, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId CLIMBER_INTAKE = new CanDeviceId(17, CANIVORE_CANBUS_NAME);

    // Intake
    private static final CanDeviceId TOP_ROLLERS_MOTOR = new CanDeviceId(18, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId RIGHT_VERTICAL_ROLLERS_MOTOR = new CanDeviceId(19, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId LEFT_VERTICAL_ROLLERS_MOTOR = new CanDeviceId(20, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_CAN_RANGE = new CanDeviceId(1, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_CAN_RANGE = new CanDeviceId(2, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_CAN_RANGE = new CanDeviceId(3, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_CAN_RANGE = new CanDeviceId(4, CANIVORE_CANBUS_NAME);

    // Switch
    public static final int NEUTRAL_MODE_SWITCH_ID = 9;
    public static final int HOME_BUTTON_ID = 8;

    // Color sensor
    public static final int CANANDCOLOR_ID = 1;

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.95);

    private static final double DRIVE_MOTOR_PINION_TEETH_COUNT = 12.0;
    /**
     * Ratio between the drive motor shaft and the output shaft the wheel is mounted on.
     */
    private static final double DRIVE_GEAR_RATIO =
            (54.0 / DRIVE_MOTOR_PINION_TEETH_COUNT) * (25.0 / 32.0) * (30.0 / 15.0);

    /**
     * Ratio between the steer motor shaft and the steer output shaft.
     */
    private static final double STEER_GEAR_RATIO = 301.0 / 9.0;

    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = 54.0 / DRIVE_MOTOR_PINION_TEETH_COUNT;

    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(22.75);

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = Units.inchesToMeters(20.75);

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND =
            Units.feetToMeters(15.0) * (425.0 / 63.0) / DRIVE_GEAR_RATIO;

    /**
     * The maximum angular speed of the robot in radians per second.
     * If set to 0, the value is calculated using the max speed in meters per second
     * and the wheelbase radius.
     */

    // CANcoder offsets of the swerve modules - bevel gears pointing left of the robot
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = -0.30517578125 + 0.5;

    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = -0.008544921875;
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = -0.341064453125 + 0.5;
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = 0.100830078125 - 0.5;

    private static final int GYRO_MOUNTING_ANGLE = 0;
    private static final double GYRO_ERROR = -0.34;

    /**
     * Path-following constants
     */
    private static final double PATHPLANNER_FOLLOW_PATH_TRANSLATION_KP = 0.0;

    private static final double PATHPLANNER_FOLLOW_PATH_TRANSLATION_KI = 0.0;
    private static final double PATHPLANNER_FOLLOW_PATH_TRANSLATION_KD = 0.0;
    private static final double PATHPLANNER_FOLLOW_PATH_ROTATION_KP = 0.0;
    private static final double PATHPLANNER_FOLLOW_PATH_ROTATION_KI = 0.0;
    private static final double PATHPLANNER_FOLLOW_PATH_ROTATION_KD = 0.0;

    private static final double CHOREO_FOLLOW_PATH_TRANSLATION_KP = 0.0;
    private static final double CHOREO_FOLLOW_PATH_TRANSLATION_KI = 0.0;
    private static final double CHOREO_FOLLOW_PATH_TRANSLATION_KD = 0.0;
    private static final double CHOREO_FOLLOW_PATH_ROTATION_KP = 0.0;
    private static final double CHOREO_FOLLOW_PATH_ROTATION_KI = 0.0;
    private static final double CHOREO_FOLLOW_PATH_ROTATION_KD = 0.0;

    // Robot configuration
    private final PortConfiguration portConfiguration;

    private final SwerveDrivetrainConstants swerveDrivetrainConstants;

    private final SwerveModuleConstants[] moduleConstants;

    private final List<CameraConfiguration> cameraConfigurations;

    private final FollowPathConfiguration pathplannerFollowPathConfiguration;

    private final FollowPathConfiguration choreoFollowPathConfiguration;

    private final ArmConfiguration armConfiguration;

    public Spectre() {
        portConfiguration = new PortConfiguration()
                .withCANBus(CANIVORE_CANBUS_NAME)
                .withCandleID(CANDLE_ID)
                .withExtensionOneID(EXTENSION_ONE_MOTOR)
                .withExtensionTwoID(EXTENSION_TWO_MOTOR)
                .withExtensionThreeID(EXTENSION_THREE_MOTOR)
                .withShoulderOneID(SHOULDER_ONE_MOTOR)
                .withShoulderTwoID(SHOULDER_TWO_MOTOR)
                .withShoulderThreeID(SHOULDER_THREE_MOTOR)
                .withWristID(WRIST_MOTOR)
                .withIntakeTopMotorID(TOP_ROLLERS_MOTOR)
                .withLeftIntakeVerticalMotorID(LEFT_VERTICAL_ROLLERS_MOTOR)
                .withRightIntakeVerticalMotorID(RIGHT_VERTICAL_ROLLERS_MOTOR)
                .withIntakeBackCANRangeID(BACK_CAN_RANGE)
                .withIntakeFrontCANRangeID(FRONT_CAN_RANGE)
                .withIntakeFrontRightCANRangeID(FRONT_RIGHT_CAN_RANGE)
                .withIntakeFrontLeftCANRangeID(FRONT_LEFT_CAN_RANGE)
                .withClimberIntakeID(CLIMBER_INTAKE)
                .withCarriageID(CLIMBER_CARRIAGE)
                .withHomeButtonID(HOME_BUTTON_ID)
                .withNeutralModeSwitchID(NEUTRAL_MODE_SWITCH_ID)
                .withCanandcolorID(CANANDCOLOR_ID);

        Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseYaw = GYRO_MOUNTING_ANGLE;
        pigeon2Configuration.GyroTrim.GyroScalarZ = GYRO_ERROR;

        swerveDrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(CANIVORE_CANBUS_NAME)
                .withPigeon2Id(GYRO.getDeviceNumber())
                .withPigeon2Configs(pigeon2Configuration);

        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 50.0;
        driveConfigs.CurrentLimits.StatorCurrentLimit = 100.0;

        var steerConfigs = new TalonFXConfiguration();
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.SupplyCurrentLimit = 30.0;
        steerConfigs.CurrentLimits.StatorCurrentLimit = 90.0;

        moduleConstants = new SwerveModuleConstants[4];
        moduleConstants[0] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.1238, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
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
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.1238, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
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
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.1238, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
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
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.1238, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        CameraConfiguration frontLeftConfiguration = new CameraConfiguration("limelight-fl");
        frontLeftConfiguration.LimelightHeightOffsetMeters = Units.inchesToMeters(8.479);
        frontLeftConfiguration.LimelightLengthOffsetMeters = Units.inchesToMeters(2.75);
        frontLeftConfiguration.LimelightWidthOffsetMeters = Units.inchesToMeters(-6.75);
        frontLeftConfiguration.LimelightMountingYawRadians = Units.degreesToRadians(0);
        frontLeftConfiguration.LimelightDistanceScalarValue = (24.25 / 24.06); // (19.75 / 19.75); // (19.75 / 22.38);
        frontLeftConfiguration.limelightLocation = CameraConfiguration.Location.FRONT_LEFT;
        CameraConfiguration frontRightConfiguration = new CameraConfiguration("limelight-fr");
        frontRightConfiguration.LimelightHeightOffsetMeters = Units.inchesToMeters(8.479);
        frontRightConfiguration.LimelightLengthOffsetMeters = Units.inchesToMeters(2.75);
        frontRightConfiguration.LimelightWidthOffsetMeters = Units.inchesToMeters(6.75);
        frontRightConfiguration.LimelightMountingYawRadians = Units.degreesToRadians(0);
        frontRightConfiguration.LimelightDistanceScalarValue = (24.25 / 24.24); // (19.75 / 20.05); // (19.75 / 22.68);
        frontRightConfiguration.limelightLocation = CameraConfiguration.Location.FRONT_RIGHT;
        CameraConfiguration backLeftConfiguration = new CameraConfiguration("limelight-bl");
        backLeftConfiguration.LimelightHeightOffsetMeters = Units.inchesToMeters(8.440);
        backLeftConfiguration.LimelightLengthOffsetMeters = Units.inchesToMeters(11.199);
        backLeftConfiguration.LimelightWidthOffsetMeters = Units.inchesToMeters(-6.75);
        backLeftConfiguration.LimelightMountingYawRadians = Units.degreesToRadians(180);
        backLeftConfiguration.LimelightDistanceScalarValue = (10.25 / 10.77); // (5.75 / 6.21); // (5.75 / 7.49);
        backLeftConfiguration.limelightLocation = CameraConfiguration.Location.BACK_LEFT;
        CameraConfiguration backRightConfiguration = new CameraConfiguration("limelight-br");
        backRightConfiguration.LimelightHeightOffsetMeters = Units.inchesToMeters(8.440);
        backRightConfiguration.LimelightLengthOffsetMeters = Units.inchesToMeters(11.199);
        backRightConfiguration.LimelightWidthOffsetMeters = Units.inchesToMeters(6.75);
        backRightConfiguration.LimelightMountingYawRadians = Units.degreesToRadians(180);
        backRightConfiguration.LimelightDistanceScalarValue = (10.25 / 10.88); // (5.75 / 6.72); // (5.75 / 8.09);
        backRightConfiguration.limelightLocation = CameraConfiguration.Location.BACK_RIGHT;
        cameraConfigurations =
                List.of(frontLeftConfiguration, frontRightConfiguration, backLeftConfiguration, backRightConfiguration);

        armConfiguration = new ArmConfiguration()
                .withExtensionkP(5.0)
                .withExtensionkI(0)
                .withExtensionkD(0)
                .withExtensionkS(0)
                .withShoulderkP(5.0)
                .withShoulderkI(0)
                .withShoulderkD(0)
                .withShoulderkS(0)
                .withWristkP(5.0)
                .withWristkI(0)
                .withWristkD(0)
                .withWristkS(0);

        pathplannerFollowPathConfiguration = new FollowPathConfiguration()
                .withTranslationKp(PATHPLANNER_FOLLOW_PATH_TRANSLATION_KP)
                .withTranslationKi(PATHPLANNER_FOLLOW_PATH_TRANSLATION_KI)
                .withTranslationKd(PATHPLANNER_FOLLOW_PATH_ROTATION_KD)
                .withRotationKp(PATHPLANNER_FOLLOW_PATH_ROTATION_KP)
                .withRotationKi(PATHPLANNER_FOLLOW_PATH_ROTATION_KI)
                .withRotationKd(PATHPLANNER_FOLLOW_PATH_ROTATION_KD);

        choreoFollowPathConfiguration = new FollowPathConfiguration()
                .withTranslationKp(CHOREO_FOLLOW_PATH_TRANSLATION_KP)
                .withTranslationKi(CHOREO_FOLLOW_PATH_TRANSLATION_KI)
                .withTranslationKd(CHOREO_FOLLOW_PATH_ROTATION_KD)
                .withRotationKp(CHOREO_FOLLOW_PATH_ROTATION_KP)
                .withRotationKi(CHOREO_FOLLOW_PATH_ROTATION_KI)
                .withRotationKd(CHOREO_FOLLOW_PATH_ROTATION_KD);
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
        return cameraConfigurations;
    }

    @Override
    public FollowPathConfiguration getChoreoPathConfiguration() {
        return choreoFollowPathConfiguration;
    }

    @Override
    public FollowPathConfiguration getPathPlannerPathConfiguration() {
        return pathplannerFollowPathConfiguration;
    }

    @Override
    public ArmConfiguration getArmConfiguration() {
        return armConfiguration;
    }
}
