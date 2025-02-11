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

  // elevator code
  // place holders
  public static int LEFT_ELEVATOR_ID = 40;
  public static int RIGHT_ELEVATOR_ID = 41;

  public static double ELEVATOR_P = 0;
  public static double ELEVATOR_I = 0;
  public static double ELEVATOR_D = 0;

  // This equals the gear ratio of the elevator motor times pi times the diamitor of the string
  // housing
  // 4 is the gear ratio
  // 2 is the diamitor of the string housing
  public static double ROTATIONS_TO_INCHES = 4 * Math.PI * 2;

  // Transition Code
  public static final int TRANSITION_ID = 31;

  // Intake Code
  public static final int INTAKE_MOTOR_ID = 33;
  public static final int PIVOT_MOTOR_ID = 32;

  public static final int ARM_MOTOR_ID = 0;
  public static final int ARM_PIVOT_PORT = 0;
  public static final int PIVOT_ANGLE_OFFSET = 0;

  public static double TransitionP = 0.0;
  public static double TransitionI = 0.0;
  public static double TransitionD = 0.0;
}
