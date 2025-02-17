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

import edu.wpi.first.wpilibj.RobotBase;

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

  // Intake Code
  public static final int INTAKE_MOTOR_ID = 33;
  public static final int PIVOT_MOTOR_ID = 32;

  public static final int ARM_MOTOR_ID = 44;
  public static final int ARM_PIVOT_PORT = 0;
  public static final int PIVOT_ANGLE_OFFSET = -90;
  public static final double ARM_GEAR_RATIO = 43.95;
  public static final double ARM_MAX_VOLTAGE = 1.0; // Set this to 8 for competition

  public static double TRANSITION_P = 0.0;
  public static double TRANSITION_I = 0.0;
  public static double TRANSITION_D = 0.0;

  public static class Setpoints {
    public static final double HOME_HEIGHT_IN = 1.0;
    public static final double L2_HEIGHT_IN = 1.0;
    public static final double L3_HEIGHT_IN = 13.0;
    public static final double L4_HEIGHT_IN = 54.0;

    public static final double HOME_ANGLE = -90;
    public static final double L2_ANGLE = -35;
    public static final double L3_ANGLE = -35;
    public static final double L4_ANGLE = 0;

    public static final double POSITION_TOLERANCE = 1.0;
    public static final double ANGLE_TOLERANCE = 3.0;
  }
}
