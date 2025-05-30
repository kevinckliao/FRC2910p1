package org.frc2910.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc2910.robot.config.CameraConfiguration;
import org.frc2910.robot.constants.Constants;
import org.frc2910.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private final List<AprilTagObservation> aprilTagObservations = new ArrayList<>();
    private Pose2d robotToFieldFromSwerveDriveOdometry = new Pose2d();
    private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();
    private Constants.SuperstructureConstants.ScoringDirection scoringDirection =
            Constants.SuperstructureConstants.ScoringDirection.BACK;

    public record AprilTagObservation(
            String cameraName, CameraConfiguration.Location location, int tagId, Pose2d robotPoseFromCamera) {}

    public record SwerveDriveObservation(Pose2d robotPose, ChassisSpeeds robotSpeeds) {}

    public void addVisionObservation(AprilTagObservation... observations) {
        aprilTagObservations.clear();

        for (var observation : observations) {
            if ((observation.location() == CameraConfiguration.Location.FRONT_LEFT
                    || observation.location() == CameraConfiguration.Location.FRONT_RIGHT)) {
                if (observation.tagId == getValidTagIDsFromClosest60DegreeRotation().FRONT_ID) {
                    this.scoringDirection = Constants.SuperstructureConstants.ScoringDirection.FRONT;
                }
            } else if ((observation.location() == CameraConfiguration.Location.BACK_LEFT
                    || observation.location() == CameraConfiguration.Location.BACK_RIGHT)) {
                if (observation.tagId == getValidTagIDsFromClosest60DegreeRotation().BACK_ID) {
                    this.scoringDirection = Constants.SuperstructureConstants.ScoringDirection.BACK;
                }
            }

            Logger.recordOutput("RobotState/ScoringDirection", scoringDirection);

            aprilTagObservations.add(observation);
        }
    }

    public void addPoseObservation(SwerveDriveObservation observation) {
        this.robotToFieldFromSwerveDriveOdometry = observation.robotPose;
        this.robotChassisSpeeds = observation.robotSpeeds;
    }

    public List<AprilTagObservation> getAprilTagObservations() {
        return aprilTagObservations;
    }

    public Pose2d getRobotPoseFromSwerveDriveOdometry() {
        return robotToFieldFromSwerveDriveOdometry;
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return robotChassisSpeeds;
    }

    public Constants.SuperstructureConstants.ScoringDirection getScoringDirection() {
        return scoringDirection;
    }

    public Constants.ReefConstants.ScoringCoralMappingRotationToTagID getValidTagIDsFromClosest60DegreeRotation() {
        return getValidTagIDsFromClosest60DegreeRotation(getClosest60Degrees());
    }

    public Constants.ReefConstants.ScoringCoralMappingRotationToTagID getValidTagIDsFromClosest60DegreeRotation(
            Rotation2d closest60Degrees) {
        var ids = FieldConstants.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAngleToTagIDsMap.get(closest60Degrees)
                : Constants.ReefConstants.redAllianceAngleToTagIDsMap.get(closest60Degrees);
        Logger.recordOutput("RobotState/ValidTagIdsFromRotation/FrontID", ids.FRONT_ID);
        Logger.recordOutput("RobotState/ValidTagIdsFromRotation/BackID", ids.BACK_ID);
        return ids;
    }

    public Rotation2d getClosest60Degrees() {
        double[] list = {60, 120, 180, -60, -120, 0};
        double desiredRotation = 0;
        for (double e : list) {
            var rotation = Rotation2d.fromDegrees(e);
            if (robotToFieldFromSwerveDriveOdometry
                                    .getRotation()
                                    .minus(rotation)
                                    .getDegrees()
                            < 30.0
                    && robotToFieldFromSwerveDriveOdometry
                                    .getRotation()
                                    .minus(rotation)
                                    .getDegrees()
                            >= -30) {
                desiredRotation = e;
            }
        }
        Logger.recordOutput("RobotState/Closest60DegreeAngle", desiredRotation);
        return Rotation2d.fromDegrees(desiredRotation);
    }

    public Pair<Rotation2d, Constants.SuperstructureConstants.ScoringDirection>
            getClosestRotationToFaceNearestReefFace() {
        return getClosestRotationToFaceNearestReefFace(false);
    }

    public Pair<Rotation2d, Constants.SuperstructureConstants.ScoringDirection> getClosestRotationToFaceNearestReefFace(
            boolean useFront) {
        var pose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
        int correctTagID = getClosestTagId();
        Logger.recordOutput("ClosestTagId", correctTagID);

        var mapToUse = FieldConstants.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAngleToTagIDsMap
                : Constants.ReefConstants.redAllianceAngleToTagIDsMap;
        var pairToReturn = Pair.of(Rotation2d.kZero, Constants.SuperstructureConstants.ScoringDirection.FRONT);
        Rotation2d frontScoreRotation = new Rotation2d();
        Rotation2d backScoreRotation = new Rotation2d();

        for (Map.Entry<Rotation2d, Constants.ReefConstants.ScoringCoralMappingRotationToTagID> entry :
                mapToUse.entrySet()) {
            if (entry.getValue().FRONT_ID == correctTagID) {
                frontScoreRotation = entry.getKey();
            } else if (entry.getValue().BACK_ID == correctTagID) {
                backScoreRotation = entry.getKey();
            }
        }

        if (useFront) {
            pairToReturn = Pair.of(frontScoreRotation, Constants.SuperstructureConstants.ScoringDirection.FRONT);
        } else if (pose.getRotation().minus(backScoreRotation).getDegrees() >= -90
                && pose.getRotation().minus(backScoreRotation).getDegrees() <= 90) {
            pairToReturn = Pair.of(backScoreRotation, Constants.SuperstructureConstants.ScoringDirection.BACK);
        } else {
            pairToReturn = Pair.of(frontScoreRotation, Constants.SuperstructureConstants.ScoringDirection.FRONT);
        }
        return pairToReturn;
    }

    public int getClosestTagId() {
        var pose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
        List<Pose2d> possiblePoses = List.of();
        int correctTagID = 0;

        if (FieldConstants.isBlueAlliance()) {
            possiblePoses = Constants.ReefConstants.blueAlliancePoseToTagIDsMap.keySet().stream()
                    .toList();
            correctTagID = Constants.ReefConstants.blueAlliancePoseToTagIDsMap.get(pose.nearest(possiblePoses));

        } else {
            possiblePoses = Constants.ReefConstants.redAlliancePoseToTagIDsMap.keySet().stream()
                    .toList();
            correctTagID = Constants.ReefConstants.redAlliancePoseToTagIDsMap.get(pose.nearest(possiblePoses));
        }

        return correctTagID;
    }
}
