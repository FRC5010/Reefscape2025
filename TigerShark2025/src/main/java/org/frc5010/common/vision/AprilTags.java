// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class AprilTags {
  /** The default field layout */
  public static AprilTagFieldLayout aprilTagFieldLayout;
  /** The default room layout */
  public static AprilTagFieldLayout aprilTagRoomLayout;
  /** A map of poses to April Tag IDs for reverse lookup */
  public static Map<Pose2d, Integer> poseToID = new HashMap<>();

  /** An set of enum constants that define the April Tag positions for a custom field layout. */
  public static enum AprilTag5010 {
    /** ID 0 */
    ID0(0, 0, 0, 0, 0),
    /** ID 1 */
    ID1(0, 0, 0, 0, 0),
    /** ID 2 */
    ID2(0, 0, 0, 0, 0),
    /** ID 3 */
    ID3(0, 0, 0, 0, 0),
    /** ID 4 */
    ID4(0, 0, 0, 0, 0),
    /** ID 5 */
    ID5(0, 0, 0, 0, 0),
    /** ID 6 */
    ID6(0, 0, 0, 0, 0),
    /** ID 7 */
    ID7(0, 0, 0, 0, 0),
    /** ID 8 */
    ID8(0, 0, 0, 0, 0),
    /** ID 9 */
    ID9(0, 0, 0, 0, 0),
    /** ID 10 */
    ID10(0, 0, 0, 0, 0),
    /** ID 11 */
    ID11(0, 0, 0, 0, 0),
    /** ID 12 */
    ID12(0, 0, 0, 0, 0),
    /** ID 13 */
    ID13(0, 0, 0, 0, 0),
    /** ID 14 */
    ID14(0, 0, 0, 0, 0),
    /** ID 15 */
    ID15(0, 0, 0, 0, 0),
    /** ID 16 */
    ID16(0, 0, 0, 0, 0),
    /** ID 17 */
    ID17(0, 0, 0, 0, 0),
    /** ID 18 */
    ID18(0, 0, 0, 0, 0),
    /** ID 19 */
    ID19(0, 0, 0, 0, 0),
    /** ID 20 */
    ID20(0, 0, 0, 0, 0),
    /** ID 21 */
    ID21(0, 0, 0, 0, 0),
    /** ID 22 */
    ID22(0, 0, 0, 0, 0),
    /** ID 23 */
    ID23(
        "C102B Door",
        Units.inchesToMeters(274.5),
        Units.inchesToMeters(455.625),
        Units.inchesToMeters(69.75),
        Units.degreesToRadians(180),
        0),
    /** ID 24 */
    ID24(
        "Broken TV",
        Units.inchesToMeters(0),
        Units.inchesToMeters(189.625),
        Units.inchesToMeters(69.75),
        0,
        0),
    /** ID 25 */
    ID25(
        "Corner Cabinet 2",
        Units.inchesToMeters(36.875),
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(70.875),
        Units.degreesToRadians(90),
        Units.degreesToRadians(0)),
    /** ID 26 */
    ID26(
        "Corner Cabinet 1",
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(20.375),
        Units.inchesToMeters(70.875),
        Units.degreesToRadians(0),
        Units.degreesToRadians(0)),
    /** ID 27 */
    ID27(
        "Belts Cabinet",
        Units.inchesToMeters(14.25),
        Units.inchesToMeters(116.25),
        Units.inchesToMeters(70.875),
        Units.degreesToRadians(0),
        Units.degreesToRadians(0)),
    /** ID 28 */
    ID28("Motor Cabinet", Units.inchesToMeters(14.25), Units.inchesToMeters(0), 0, 0, 0),
    /** ID 29 */
    ID29(
        "Tack Board 1",
        Units.inchesToMeters(94.125),
        Units.inchesToMeters(.75),
        Units.inchesToMeters(69.75),
        Units.degreesToRadians(90),
        0);

    // --------------------------------------
    // The rest of this is just boiler-plate for setting up the aprilTagPoses array
    // with all of the defined april tags.
    /** The pose of the AprilTag */
    public Pose3d pose;
    /** String that describes the AprilTag */
    public String fieldDescriptor;

    private AprilTag5010(double xPos, double yPos, double zPos, double pitch, double yaw) {
      this.fieldDescriptor = this.name() + " ID:" + this.ordinal();
      pose =
          new Pose3d(
              new Translation3d(xPos, yPos, zPos),
              new Rotation3d(0.0, Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)));
    }

    private AprilTag5010(
        String fieldDescriptor, double xPos, double yPos, double zPos, double pitch, double yaw) {
      this.fieldDescriptor = fieldDescriptor + " ID:" + this.ordinal();
      pose =
          new Pose3d(
              new Translation3d(xPos, yPos, zPos),
              new Rotation3d(0.0, Units.degreesToRadians(pitch), Units.degreesToRadians(yaw)));
    }
  }

  static {
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);

      List<AprilTag> aprilTagPoses = new ArrayList<>();
      for (AprilTag5010 aprilTag : AprilTag5010.values()) {
        aprilTagPoses.add(new AprilTag(aprilTag.ordinal(), aprilTag.pose));
      }

      aprilTagRoomLayout =
          new AprilTagFieldLayout(aprilTagPoses, Units.feetToMeters(30), Units.feetToMeters(30));

    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
  }
}
