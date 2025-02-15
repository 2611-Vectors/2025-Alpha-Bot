// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final PIDController elevatorPID = new PIDController(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
  private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.0, 0.45, 0.0);

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(Constants.LEFT_ELEVATOR_ID);
    rightMotor = new TalonFX(Constants.RIGHT_ELEVATOR_ID);

    PhoenixUtil.configMotor(leftMotor, true, elevatorPID, elevatorFF, NeutralModeValue.Brake);
    PhoenixUtil.configMotor(rightMotor, false, elevatorPID, elevatorFF, NeutralModeValue.Brake);
  }

  /** Function for voltage control */
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  /**
   * Sets the Elevator to a target position and should be called periodically unit
   * are in inches
   */
  public void setElevatorPosition(double target) {
    Logger.recordOutput("Elevator/TargetPosition", target);

    // TODO Add a simulator update

    double pidPart = elevatorPID.calculate(getLeftElevatorPosition(), target);
    double ffPart = elevatorFF.calculate(target);

    setVoltage(
        MathUtil.clamp(
            pidPart + ffPart, -Constants.ELEVATOR_MAX_VOLTAGE, Constants.ELEVATOR_MAX_VOLTAGE));
  }

  /**
   * Function to get the Elevator position uses the left by default but graphs
   * both the left and
   * right encoder values
   */
  public double getLeftElevatorPosition() {
    return leftMotor.getPosition().getValueAsDouble() * Constants.ROTATIONS_TO_INCHES;
  }

  public double getRightElevatorPosition() {
    return rightMotor.getPosition().getValueAsDouble() * Constants.ROTATIONS_TO_INCHES;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Telemetry for position might be useful for the simulator hint hint hint
    Logger.recordOutput("Elevator/LeftEncoder", getLeftElevatorPosition());
    Logger.recordOutput("Elevator/RightEncoder", getRightElevatorPosition());
  }
}
