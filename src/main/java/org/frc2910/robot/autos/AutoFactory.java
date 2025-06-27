package org.frc2910.robot.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.frc2910.robot.RobotContainer;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.constants.Constants;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.subsystems.Superstructure;

import java.util.Set;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
class AutoFactory {
    private final DriverStation.Alliance alliance;

    private final RobotContainer robotContainer;

    private final Choreo.TrajectoryCache trajectoryCache;

    private final double DISTANCE_TO_MOVE_ARM_UP = Units.inchesToMeters(48.0);

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;

        trajectoryCache = new Choreo.TrajectoryCache();
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Pair<Pose2d, Command> createIdleCommand() {
        return Pair.of(FieldConstants.getFarLeftStartingPose(alliance), IDLE_COMMAND);
    }

    Pair<Pose2d, Command> createJKLABAuto() {
        var initialPose = FieldConstants.getLeftStartingPose(alliance);
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.IJ,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                10.0),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.feetToMeters(10.0)),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark1()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(4.0)),
                                Set.of()),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_LEFT_L4),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark2()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.5)),
                                Set.of()),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_RIGHT_L4),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createEDCBAAuto() {
        var initialPose = FieldConstants.getRightStartingPose(alliance);
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.EF,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                10.0),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.feetToMeters(10.0)),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark3()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(4.0)),
                                Set.of()),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_RIGHT_L4),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark2()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.5)),
                                Set.of()),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_LEFT_L4),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createMarkTestAuto() {

        var initialPose = FieldConstants.getDesiredFinalScoringPoseForCoral(
                18,
                Constants.SuperstructureConstants.ScoringSide.RIGHT,
                Constants.SuperstructureConstants.ScoringDirection.BACK);

        return Pair.of(
                initialPose,
                Commands.sequence(new DeferredCommand(
                        () -> (driveToPoint(getIntakePose(FieldConstants.getMark2()), Units.feetToMeters(3.0))
                                        .alongWith(setState(Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND)))
                                .raceWith(waitForCoralPickup()),
                        Set.of())));
    }

    Pair<Pose2d, Command> createABBAAuto() {
        var initialTrajectory = trajectoryCache
                .loadTrajectory(trajectoryName(Location.FAR_LEFT_STARTING, Location.A_CORAL))
                .get();

        var initialPose = initialTrajectory.getInitialPose(false).get();

        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB,
                                (Trajectory<SwerveSample>) initialTrajectory,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4),
                        followAndIntakeFromMark(Units.feetToMeters(3.0), FieldConstants.getMark2()),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.AB,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.feetToMeters(8.0)),
                        followAndIntakeFromMark(Units.feetToMeters(5.0), FieldConstants.getMark3()),
                        followThenScoreWithMinimumReleaseTime(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_RIGHT_L3),
                        followAndIntakeFromMark(Units.feetToMeters(5.0), FieldConstants.getMark1()),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_LEFT_L3),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createBABAAuto() {
        var initialTrajectory = trajectoryCache
                .loadTrajectory(trajectoryName(Location.FAR_RIGHT_STARTING, Location.B_CORAL))
                .get();

        var initialPose = initialTrajectory.getInitialPose(false).get();

        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB,
                                (Trajectory<SwerveSample>) initialTrajectory,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4),
                        followAndIntakeFromMark(Units.feetToMeters(3.0), FieldConstants.getMark2()),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.AB,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.feetToMeters(8.0)),
                        followAndIntakeFromMark(Units.feetToMeters(5.0), FieldConstants.getMark1()),
                        followThenScoreWithMinimumReleaseTime(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_RIGHT_L3),
                        followAndIntakeFromMark(Units.feetToMeters(5.0), FieldConstants.getMark3()),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.AB, Superstructure.WantedSuperState.SCORE_LEFT_L3),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createIKLJAuto() {
        var offsetID = alliance == DriverStation.Alliance.Blue
                ? Constants.ReefConstants.blueAllianceReefFacesToIds.get(Constants.ReefConstants.ReefFaces.IJ)
                : Constants.ReefConstants.redAllianceReefFacesToIds.get(Constants.ReefConstants.ReefFaces.IJ);
        var initialPose = FieldConstants.getLeftStartingPose(alliance);
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.IJ,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.inchesToMeters(72.0),
                                Units.feetToMeters(8.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.inchesToMeters(72.0),
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(12.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.inchesToMeters(72.0),
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(12.0)),
                        driveToPoint(
                                        FieldConstants.getDesiredPointToDriveToForCoralScoring(
                                                offsetID,
                                                Constants.SuperstructureConstants.ScoringSide.RIGHT,
                                                Constants.SuperstructureConstants.ScoringDirection.BACK,
                                                Units.inchesToMeters(30.0)),
                                        15.0)
                                .until(() -> robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint()
                                        < Units.inchesToMeters(80.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.IJ,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.inchesToMeters(60.0),
                                Units.feetToMeters(10.0)),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createFDCEAuto() {
        var offsetID = alliance == DriverStation.Alliance.Blue
                ? Constants.ReefConstants.blueAllianceReefFacesToIds.get(Constants.ReefConstants.ReefFaces.EF)
                : Constants.ReefConstants.redAllianceReefFacesToIds.get(Constants.ReefConstants.ReefFaces.EF);
        var initialPose = FieldConstants.getRightStartingPose(alliance);
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.EF,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.inchesToMeters(72.0),
                                Units.feetToMeters(8.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.inchesToMeters(72.0),
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(12.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.inchesToMeters(72.0),
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(12.0)),
                        driveToPoint(
                                        FieldConstants.getDesiredPointToDriveToForCoralScoring(
                                                offsetID,
                                                Constants.SuperstructureConstants.ScoringSide.LEFT,
                                                Constants.SuperstructureConstants.ScoringDirection.BACK,
                                                Units.inchesToMeters(30.0)),
                                        15.0)
                                .until(() -> robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint()
                                        < Units.inchesToMeters(80.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.EF,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.inchesToMeters(60.0),
                                Units.feetToMeters(10.0)),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createKLAAuto() {
        var initialPose = FieldConstants.getFarLeftStartingPose(alliance);
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                DISTANCE_TO_MOVE_ARM_UP),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark1()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.5)),
                                Set.of()),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.feetToMeters(6.0)),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark2()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.0)),
                                Set.of()),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.AB,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.feetToMeters(6.0)),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark3()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.5)),
                                Set.of()),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    Pair<Pose2d, Command> createDCBAuto() {
        var initialPose = FieldConstants.getFarRightStartingPose(alliance);
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark3()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.5)),
                                Set.of()),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.SCORE_LEFT_L4,
                                Units.feetToMeters(6.0)),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark2()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.0)),
                                Set.of()),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.AB,
                                Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                                Units.feetToMeters(6.0)),
                        new DeferredCommand(
                                () -> followThenIntake(
                                        getIntakePose(FieldConstants.getMark1()),
                                        Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND,
                                        Units.feetToMeters(3.5)),
                                Set.of()),
                        setState(Superstructure.WantedSuperState.DEFAULT_STATE)));
    }

    private String trajectoryName(final Location start, final Location end) {
        var name = "%S_TO_%S".formatted(start, end);
        var x = "%S_%S".formatted(alliance, name);
        System.out.println("%S_%S".formatted(alliance, name));
        return x;
    }

    Command setState(Superstructure.WantedSuperState state) {
        return robotContainer.getSuperstructure().setStateCommand(state);
    }

    Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        return new InstantCommand(() -> robotContainer.getSwerveSubsystem().setDesiredChoreoTrajectory(trajectory));
    }

    Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return new InstantCommand(() -> robotContainer
                        .getSwerveSubsystem()
                        .setDesiredPoseForDriveToPointWithConstraints(point, maxVelocityOutputForDriveToPoint, 1.0))
                .andThen(new WaitUntilCommand(
                        () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
    }

    Command driveToPointWithUnconstrainedMaxVelocity(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return new InstantCommand(() -> robotContainer
                        .getSwerveSubsystem()
                        .setDesiredPoseForDriveToPointWithConstraints(
                                point, maxVelocityOutputForDriveToPoint, Double.NaN))
                .andThen(new WaitUntilCommand(
                        () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Superstructure.WantedSuperState scoreState,
            double distanceFromEndOfPathtoMoveArmUp,
            double maxVelocity) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, maxVelocity)
                        .alongWith(new WaitUntilCommand(() ->
                                        robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint()
                                                < distanceFromEndOfPathtoMoveArmUp)
                                .andThen(setState(scoreState)))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Superstructure.WantedSuperState scoreState,
            double distanceFromEndOfPathtoMoveArmUp) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(8.0))
                        .alongWith(new WaitUntilCommand(() ->
                                        robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint()
                                                < distanceFromEndOfPathtoMoveArmUp)
                                .andThen(setState(scoreState)))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScoreWithNonDefaultMaxVelocity(
            Constants.ReefConstants.ReefFaces reefFaces,
            Superstructure.WantedSuperState scoreState,
            double maxVelocity) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, maxVelocity).alongWith(setState(scoreState))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScoreWithMinimumReleaseTime(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
                .andThen(new WaitCommand(0.5).andThen(waitForCoralRelease()).raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Trajectory<SwerveSample> path,
            Superstructure.WantedSuperState scoreState) {
        var noCoralState = (scoreState == Superstructure.WantedSuperState.SCORE_LEFT_L2
                        || scoreState == Superstructure.WantedSuperState.SCORE_LEFT_L3
                        || scoreState == Superstructure.WantedSuperState.SCORE_LEFT_L4)
                ? Superstructure.WantedSuperState.FORCE_RELOCALIZE_LEFT
                : Superstructure.WantedSuperState.FORCE_RELOCALIZE_RIGHT;
        return followThenScore(reefFaces, path, scoreState, noCoralState);
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Trajectory<SwerveSample> path,
            Superstructure.WantedSuperState scoreState,
            Superstructure.WantedSuperState noCoralState) {
        return (followTrajectory(path)
                        .andThen(new WaitUntilCommand(
                                robotContainer.getSwerveSubsystem()::isAtEndOfChoreoTrajectoryOrDriveToPoint)))
                .alongWith(new WaitUntilCommand(
                                () -> robotContainer.getSwerveSubsystem().getRobotDistanceFromChoreoEndpoint()
                                        < DISTANCE_TO_MOVE_ARM_UP)
                        .andThen(new ConditionalCommand(
                                followThenScore(reefFaces, scoreState),
                                setState(noCoralState),
                                () -> robotContainer.getSuperstructure().hasCoral())));
    }

    public Pose2d getIntakePose(Translation2d intakeLocation) {
        var angle = intakeLocation
                .minus(RobotState.getInstance()
                        .getRobotPoseFromSwerveDriveOdometry()
                        .getTranslation())
                .getAngle();
        // var translation = intakeLocation.plus(new Translation2d(2.0, angle));
        return new Pose2d(intakeLocation, angle);
    }

    private Command followThenIntakeFromStation(Pose2d intakePose, double intakeVelocity) {
        return (driveToPoint(intakePose, intakeVelocity)
                        .alongWith(setState(Superstructure.WantedSuperState.INTAKE_CORAL_FROM_STATION_STRAIGHT))
                        .andThen(Commands.waitSeconds(2.0)))
                .raceWith(waitForCoralPickup());
    }

    private Command followThenIntake(
            Pose2d intakePose, Superstructure.WantedSuperState intakeState, double intakeVelocity) {
        return (driveToPoint(intakePose, intakeVelocity).alongWith(setState(intakeState)))
                .raceWith(waitForCoralPickup());
    }

    private Command followAndIntakeFromMark(double velocity, Translation2d markLocation) {
        return new DeferredCommand(
                () -> driveToPoint(getIntakePose(markLocation), velocity)
                        .alongWith(setState(Superstructure.WantedSuperState.MARK_PUNCH)
                                .andThen(new WaitUntilCommand(() -> robotContainer
                                                        .getSwerveSubsystem()
                                                        .getDistanceFromDriveToPointSetpoint()
                                                < 0.75)
                                        .andThen(setState(Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND))))
                        .raceWith(waitForCoralPickup()),
                Set.of());
    }

    private Command waitForCoralRelease() {
        return new WaitUntilCommand(() -> !robotContainer.getSuperstructure().hasCoral());
    }

    private Command waitForCoralPickup() {
        return new WaitUntilCommand(() -> robotContainer.getSuperstructure().hasCollectedPieceInAuto());
    }

    public Pose2d getAutoScoringPose(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState superState) {
        var map = alliance == DriverStation.Alliance.Blue
                ? Constants.ReefConstants.blueAllianceReefFacesToIds
                : Constants.ReefConstants.redAllianceReefFacesToIds;
        var id = map.get(reefFaces);
        var scoringSide = (superState == Superstructure.WantedSuperState.SCORE_LEFT_L2
                        || superState == Superstructure.WantedSuperState.SCORE_LEFT_L3
                        || superState == Superstructure.WantedSuperState.SCORE_LEFT_L4)
                ? Constants.SuperstructureConstants.ScoringSide.LEFT
                : Constants.SuperstructureConstants.ScoringSide.RIGHT;
        return FieldConstants.getDesiredFinalScoringPoseForCoral(
                id, scoringSide, Constants.SuperstructureConstants.ScoringDirection.BACK);
    }
}
