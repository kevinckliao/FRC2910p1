package org.frc2910.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc2910.robot.config.CameraConfiguration;
import org.frc2910.robot.constants.FieldConstants;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import java.util.Collection;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class VisionIOLimelightTests {
    VisionIOLimelight visionIO = new VisionIOLimelight(new CameraConfiguration());

    @Nested
    class SortCornersTests {
        @Test
        public void CornerSortTest() {
            // Arrange
            final Collection<Pair<Double, Double>> corners =
                    List.of(Pair.of(2.0, 0.0), Pair.of(0.0, 2.0), Pair.of(2.0, 2.0), Pair.of(0.0, 0.0));

            // Act
            final List<Pair<Double, Double>> sortedCorners =
                    VisionIOLimelight.sortCorners(corners).stream().toList();
            final Pair<Double, Double> corner0 = sortedCorners.get(0);
            final Pair<Double, Double> corner1 = sortedCorners.get(1);
            final Pair<Double, Double> corner2 = sortedCorners.get(2);
            final Pair<Double, Double> corner3 = sortedCorners.get(3);

            // Assert
            assertEquals(0.0, corner0.getFirst());
            assertEquals(2.0, corner0.getSecond());

            assertEquals(2.0, corner1.getFirst());
            assertEquals(2.0, corner1.getSecond());

            assertEquals(2.0, corner2.getFirst());
            assertEquals(0.0, corner2.getSecond());

            assertEquals(0.0, corner3.getFirst());
            assertEquals(0.0, corner3.getSecond());
        }

        @Test
        public void CalculateHeightTest() {
            // Arrange
            final Collection<Pair<Double, Double>> corners =
                    List.of(Pair.of(2.0, 0.0), Pair.of(0.0, 2.0), Pair.of(2.0, 2.0), Pair.of(0.0, 0.0));

            // Act
            final List<Pair<Double, Double>> sortedCorners =
                    VisionIOLimelight.sortCorners(corners).stream().toList();

            final double Height = VisionIOLimelight.calculateTargetHeightInPixels(sortedCorners);

            // Assert
            assertEquals(2.0, Height);
        }

        @Test
        public void CalculateTagDistanceMeters() {
            // Arrange
            final Collection<Pair<Double, Double>> corners =
                    List.of(Pair.of(2.0, 0.0), Pair.of(0.0, 2.0), Pair.of(2.0, 2.0), Pair.of(0.0, 0.0));

            final List<Pair<Double, Double>> sortedCorners =
                    VisionIOLimelight.sortCorners(corners).stream().toList();
            final double Height = VisionIOLimelight.calculateTargetHeightInPixels(sortedCorners);

            // Act
            final double DistanceToTagMeters = visionIO.calculateDistanceToAprilTagInMetersZaneMethod(Height);

            // Assert
            assertEquals(73.0, ((double) Math.round(DistanceToTagMeters)));
        }
    }

    @Nested
    class calculateDistanceToAprilTagInMetersUsingTrigMethod {

        @ParameterizedTest
        @CsvSource({"0, 18, 2, 2", "0, 21, 0, 180", "0, 22, 0, 120", "0, 19, 0, -60", "45, 18, 45, 0", "45, 19, 45, -60"
        })
        public void TestLocalizationMethod(
                double expected, int tagId, double txInDegrees, double driveRotationInDegrees) {
            assertEquals(
                    expected,
                    calculateAngleToTag(
                                    tagId,
                                    Rotation2d.fromDegrees(txInDegrees),
                                    Rotation2d.fromDegrees(driveRotationInDegrees))
                            .getDegrees(),
                    0.03);
        }

        Rotation2d calculateAngleToTag(int tagId, Rotation2d tx, Rotation2d robotRotation) {
            var tagRotation = FieldConstants.getTagPose(tagId).getRotation().toRotation2d();
            return (tagRotation.plus(Rotation2d.k180deg)).minus(tx.unaryMinus().plus(robotRotation));
        }
    }
}
