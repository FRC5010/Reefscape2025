package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.frc5010.common.constants.FieldDimensions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

// Constants for field in blue alliance origin
// Values borrowed from 6328's code
public class FieldConstants {
  public static FieldDimensions fieldDimensions = FieldDimensions.REEFSCAPE;
  public static final Distance innerStartingLineX =
      Inches.of(299.438);
  public static final Distance algaeDiameter = Inches.of(16);

  public static class Processor {
    public static final Pose2d center =
        new Pose2d(Inches.of(235.726), Inches.of(0), Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Inches.of(345.428), Inches.of(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Inches.of(345.428), Inches.of(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Inches.of(345.428), Inches.of(199.947));

    public static final Distance deepHeight = Inches.of(3.125);
    public static final Distance shallowHeight = Inches.of(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Inches.of(33.526),
            Inches.of(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Inches.of(33.526),
            Inches.of(25.824),
            Rotation2d.fromDegrees(144.011 - 90));


    public static final Distance GuideSpacing = Inches.of(8);
    public static final Distance GuideHalfSpacing = GuideSpacing.div(2);

    /*
     * Returns the offset pose of the guide from the center face
     * 
     * @param centerFace The center face of the coral station
     * 
     */
    public static Pose2d getGuideOffsetPose(Pose2d centerFace, int guideNumber, Transform2d poseOffset) {
        int direction = guideNumber > 0 ? 1 : -1;
        direction *= -1;
        return centerFace.transformBy(
            new Transform2d(
                new Translation2d(Inches.zero(), 
                    GuideHalfSpacing.times(Math.abs(guideNumber) * 2).times(direction)),
                Rotation2d.fromDegrees(0))).transformBy(new Transform2d(Inches.of(0), Inches.zero(), new Rotation2d())).transformBy(poseOffset);
    }
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Inches.of(176.746), Inches.of(158.501));
    public static final Distance faceToZoneLine =
        Inches.of(12);

    public static final Distance centerToFace =
        Inches.of(32.526);

    public enum Side {
        AB(new Pose2d(Inches.of(144.003), Inches.of(158.500), Rotation2d.fromDegrees(180))),
        KL(new Pose2d(Inches.of(160.373), Inches.of(186.857), Rotation2d.fromDegrees(120))),
        IJ(new Pose2d(Inches.of(193.116), Inches.of(186.858), Rotation2d.fromDegrees(60))),
        GH(new Pose2d(Inches.of(209.489), Inches.of(158.502), Rotation2d.fromDegrees(0))),
        EF(new Pose2d(Inches.of(193.118), Inches.of(130.145), Rotation2d.fromDegrees(-60))),
        CD(new Pose2d(Inches.of(160.375), Inches.of(130.144), Rotation2d.fromDegrees(-120)));


        public final Pose2d centerFace;
        

        private Side(Pose2d centerFace) {
            this.centerFace = centerFace;
        }

        public Pose2d getCenterFace() {
            return centerFace;
        }

        public Pose2d getRobotPose(Transform2d centerFaceToRobotTransform) {
            return getCenterFace().transformBy(new Transform2d(Inches.of(0), Inches.zero(), new Rotation2d())).transformBy(centerFaceToRobotTransform);
        }
    }

    public static final Pose2d[] centerFaces =
        new Pose2d[6];
    public static final Map<Side, List<Map<ReefBranch, Pose3d>>> branchPositions =
        new HashMap<>();
  }



public enum ReefBranch{
    L4(Inches.of(72), Degrees.of(90)),
    L3(Inches.of(47.625), Degrees.of(35)),
    L2(Inches.of(31.875), Degrees.of(35));

    private Transform3d constructBranchTransform(Distance height, Angle pitch) {
        return new Transform3d(
            new Translation3d(Inches.zero(), Inches.zero(), height),
            new Rotation3d(Degrees.zero(), pitch, Degrees.zero()));
    }

    private ReefBranch(Distance height, Angle pitch) {
        this.height = height;
        this.pitch = pitch;
        this.branchTransform = constructBranchTransform(height, pitch);
    }

    private final Transform3d branchTransform;
    public Distance height;
    public Angle pitch;

    public Transform3d getBranchTransform() {
        return branchTransform;
    }
  }

  
}
