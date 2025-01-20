package org.frc5010.common.constants;

import com.pathplanner.lib.config.PIDConstants;

/** A class for library constants */
public class Constants {

  /** Auton constants */
  public static final class AutonConstants {
    /** Translation PID constants */
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, .1);
    /** Rotation PID constants */
    public static final PIDConstants ANGLE_PID = new PIDConstants(5, 0, .1);
  }

  public static final class Simulation {
    public static String gamePieceA = "GPA";
    public static String gamePieceB = "GPB";
  }
}
