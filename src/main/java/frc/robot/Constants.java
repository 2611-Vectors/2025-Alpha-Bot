// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int LEFT_ELEVATOR_ID = 40;
  public static final int RIGHT_ELEVATOR_ID = 41;

  public static double ELEVATOR_P = 3;
  public static double ELEVATOR_I = 0.01;
  public static double ELEVATOR_D = 0;

  public static final double ELEVATOR_MAX_VOLTAGE = 2.0; // Set this to 8 for competition

  // Constants for the elevator motor system
  public static final double ELEVATOR_GEAR_RATIO = 4.0;
  public static final double STRING_HOUSING_DIAMETER = 2.1702;

  // Conversion factor from motor rotations to inches of travel
  public static final double ROTATIONS_TO_INCHES =
      (Math.PI * STRING_HOUSING_DIAMETER) / ELEVATOR_GEAR_RATIO;
  public static final double STARTING_HEIGHT = 0;
  // Transition Code
  public static final int TRANSITION_ID = 31;

  // Intake Constants
  public static final int INTAKE_MOTOR_ID = 33;
  public static final int PIVOT_MOTOR_ID = 32;
  public static final double INTAKE_EXTENDED = 15.0;
  public static final double INTAKE_RETRACTED = 1.0;

  // Climber Constants
  public static final int CLIMB_WINCH_ID = 51;
  public static final int CLIMB_GRAB_ID = 52;

  // Arm Constants
  public static final int ARM_MOTOR_ID = 44;
  public static final int ARM_PIVOT_PORT = 0;
  public static final int PIVOT_ANGLE_OFFSET = -90;

  public static final double ARM_GEAR_RATIO = 43.95;
  public static final double ARM_MAX_VOLTAGE = 2.0; // Set this to 8 for competition
  public static final double ARM_LENGTH = 0.6;
  public static final double LOWEST_HEIGHT = 0.3;

  public static final int END_EFFECTOR_ID = 43;

  public static final double TRANSITION_P = 0.0;
  public static final double TRANSITION_I = 0.0;
  public static final double TRANSITION_D = 0.0;

  public static class Setpoints {
    public static final double HOME_HEIGHT_IN = 1.0;
    public static final double TRAVEL_HEIGHT_IN = 1.0;
    public static final double L2_HEIGHT_IN = 5.0;
    public static final double L3_HEIGHT_IN = 21.0;
    public static final double L4_HEIGHT_IN = 46.0;
    public static final double INTAKE_HEIGHT_IN = 38;

    public static final double HOME_ANGLE = -90;
    public static final double TRAVEL_ANGLE = -90;
    public static final double L2_ANGLE = -55;
    public static final double L3_ANGLE = -55;
    public static final double L4_ANGLE = -55;
    public static final double INTAKE_ANGLE = 69.1;

    // 19 inches tall

    public static final double POSITION_TOLERANCE = 1.0;
    public static final double ANGLE_TOLERANCE = 3.0;
  }

  public static class AutonConstants {
    public static final int NUMBER_OF_CHOOSERS = 3;

    public static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(0);

    public static final Pose2d START_LEFT = new Pose2d(7.200, 6.175, START_ROTATION);
    public static final Pose2d START_CENTER = new Pose2d(7.200, 4.000, START_ROTATION);
    public static final Pose2d START_RIGHT = new Pose2d(7.200, 1.900, START_ROTATION);

    public static final Pose2d AB = new Pose2d(3.2512, 4.0259, Rotation2d.fromDegrees(180));
    public static final Pose2d CD = new Pose2d(3.8707, 2.9543, Rotation2d.fromDegrees(-120));
    public static final Pose2d EF = new Pose2d(5.1079, 2.9543, Rotation2d.fromDegrees(-60));
    public static final Pose2d GH = new Pose2d(5.7274, 4.0259, Rotation2d.fromDegrees(0));
    public static final Pose2d IJ = new Pose2d(5.1079, 5.0974, Rotation2d.fromDegrees(60));
    public static final Pose2d KL = new Pose2d(3.8707, 5.0974, Rotation2d.fromDegrees(120));

    public static final double LEFT_OFFSET = 0.21; // In Meters
    public static final double RIGHT_OFFSET = -0.13; // In Meters

    public static final Pose2d R1 = new Pose2d(1.13847, 7.10903, Rotation2d.fromDegrees(120));
    public static final Pose2d R0 = new Pose2d(1.13847, .94297, Rotation2d.fromDegrees(60));

    public static final Map<Pose2d, Double> poseAngleMap = new HashMap<>();

    static {
      poseAngleMap.put(AB, 0.0);
      poseAngleMap.put(CD, -60.0);
      poseAngleMap.put(EF, -120.0);
      poseAngleMap.put(GH, -180.0);
      poseAngleMap.put(IJ, -240.0);
      poseAngleMap.put(KL, -300.0);
    }

    public static final double MAX_VELOCITY = 2.00; // 5.1
    public static final double MAX_ACCELERATION = 1.25; // 2.9
  }

  public static class FrictionConstants {
    public static final double MU_FORWARD = 1.04191; // Friction coefficient in positive X direction
    public static final double MU_BACKWARD = 0.9355; // Friction coefficient in negative X direction
    public static final double MU_LEFT = 1.0; // Friction coefficient in negative Y direction
    public static final double MU_RIGHT = 1.07467; // Friction coefficient in positive Y direction
  }

  public static class VisionConstants {
    // Apriltag Field Layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Name of the PhotonVision Reef Camera
    public static String BackRightCam = "BackRightCam";

    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToBackRightCam =
        new Transform3d(
            Units.inchesToMeters(-3.25),
            Units.inchesToMeters(-4.375),
            Units.inchesToMeters(7.5),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(180)));

    public static String BackLeftCam = "BackLeftCam";

    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToBackLeftCam =
        new Transform3d(
            Units.inchesToMeters(-11),
            Units.inchesToMeters(10.5),
            Units.inchesToMeters(35),
            new Rotation3d(0.0, Math.toRadians(-1), Math.toRadians(135)));

    public static String FrontLeftCam = "FrontLeftCam";

    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToFrontLeftCam =
        new Transform3d(
            Units.inchesToMeters(12.5),
            Units.inchesToMeters(11),
            Units.inchesToMeters(35),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(40)));

    public static String FrontRightCam = "FrontRightCam";
    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToFrontRightCam =
        new Transform3d(
            Units.inchesToMeters(11),
            Units.inchesToMeters(-10.5),
            Units.inchesToMeters(7.75), // previous value 9
            new Rotation3d(0.0, Math.toRadians(-20), Math.toRadians(160)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.1;
    public static double maxZError = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }
}
