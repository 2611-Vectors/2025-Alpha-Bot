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

  public static int LEFT_ELEVATOR_ID = 40;
  public static int RIGHT_ELEVATOR_ID = 41;

  public static double ELEVATOR_P = 4;
  public static double ELEVATOR_I = 0.01;
  public static double ELEVATOR_D = 0;

  public static double ELEVATOR_MAX_VOLTAGE = 1.0; // Set this to 8 for competition

  // Constants for the elevator motor system
  public static final double ELEVATOR_GEAR_RATIO = 4.0;
  public static final double STRING_HOUSING_DIAMETER = 2.1702;

  // Conversion factor from motor rotations to inches of travel
  public static final double ROTATIONS_TO_INCHES =
      (Math.PI * STRING_HOUSING_DIAMETER) / ELEVATOR_GEAR_RATIO;

  // Transition Code
  public static final int TRANSITION_ID = 31;

  // Intake Constants
  public static final int INTAKE_MOTOR_ID = 33;
  public static final int PIVOT_MOTOR_ID = 32;
  public static final double INTAKE_EXTENDED = 15.0;
  public static final double INTAKE_RETRACTED = 1.0;

  // Arm Constants
  public static final int ARM_MOTOR_ID = 44;
  public static final int ARM_PIVOT_PORT = 0;
  public static final int PIVOT_ANGLE_OFFSET = -90;
  public static final double ARM_GEAR_RATIO = 43.95;
  public static final double ARM_MAX_VOLTAGE = 1.0; // Set this to 8 for competition

  public static final int END_EFFECTOR_ID = 43;

  public static final double TRANSITION_P = 0.0;
  public static final double TRANSITION_I = 0.0;
  public static final double TRANSITION_D = 0.0;

  public static class Setpoints {
    public static final double HOME_HEIGHT_IN = 1.0;
    public static final double L2_HEIGHT_IN = 7.0;
    public static final double L3_HEIGHT_IN = 13.0;
    public static final double L4_HEIGHT_IN = 54.0;
    public static final double INTAKE_HEIGHT_IN = 28.50;

    public static final double HOME_ANGLE = -90;
    public static final double L2_ANGLE = -55;
    public static final double L3_ANGLE = -35;
    public static final double L4_ANGLE = 0;
    public static final double INTAKE_ANGLE = 45.0;

    public static final double POSITION_TOLERANCE = 1.0;
    public static final double ANGLE_TOLERANCE = 3.0;
  }

  public static class AutonConstants {
    public static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(0); // 180

    public static final Pose2d START_LEFT = new Pose2d(8.0, 7.29, START_ROTATION);
    public static final Pose2d START_CENTER = new Pose2d(8.0, 6.20, START_ROTATION);
    public static final Pose2d START_RIGHT = new Pose2d(8.0, 5.13, START_ROTATION);

    public static final Pose2d AB = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
    public static final Pose2d CD = new Pose2d(3.7, 2.7, Rotation2d.fromDegrees(60));
    public static final Pose2d EF = new Pose2d(5.2, 2.7, Rotation2d.fromDegrees(120));
    public static final Pose2d GH = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(180));
    public static final Pose2d IJ = new Pose2d(5.3, 5.3, Rotation2d.fromDegrees(-120));
    public static final Pose2d KL = new Pose2d(3.8, 5.3, Rotation2d.fromDegrees(-60));

    public static final double LEFT_OFFSET = 0.5; // In Meters
    public static final double RIGHT_OFFSET = -0.5; // In Meters

    public static final Pose2d R1 = new Pose2d(1.5, 6.6, Rotation2d.fromDegrees(-150));
    public static final Pose2d R0 = new Pose2d(1.5, 1.4, Rotation2d.fromDegrees(60));

    public static final Map<Pose2d, Double> poseAngleMap = new HashMap<>();

    static {
      poseAngleMap.put(AB, 0.0);
      poseAngleMap.put(CD, -60.0);
      poseAngleMap.put(EF, -120.0);
      poseAngleMap.put(GH, -180.0);
      poseAngleMap.put(IJ, -240.0);
      poseAngleMap.put(KL, -300.0);
    }

    public static final double MAX_VELOCITY = 1; // 5.1
    public static final double MAX_ACCELERATION = 0.75; // 2.9
  }

  public static class FrictionConstants {
    public static final double MU_FORWARD = 1.04191; // Friction coefficient in positive X direction
    public static final double MU_BACKWARD = 0.9355; // Friction coefficient in negative X direction
    public static final double MU_LEFT = 1.0; // Friction coefficient in negative Y direction
    public static final double MU_RIGHT = 1.0; // Friction coefficient in positive Y direction
  }

  public static class VisionConstants {
    // Apriltag Field Layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Name of the PhotonVision Reef Camera
    public static String reefCamName = "ReefTagCam";

    // Position of the PhotonVision Reef Camera
    public static Transform3d robotToReefCam =
        new Transform3d(
            Units.inchesToMeters(-12.0),
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(7.75),
            new Rotation3d(0.0, 0.0, Math.toRadians(180)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.1;

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
