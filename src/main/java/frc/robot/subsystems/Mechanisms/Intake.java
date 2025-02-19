// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final TalonFX intake = new TalonFX(Constants.INTAKE_MOTOR_ID);
  private final TalonFX pivot = new TalonFX(Constants.PIVOT_MOTOR_ID);

  private TunablePIDController intakePID =
      new TunablePIDController(0.0, 0.0, 0.0, "/Intake/IntakePID/");
  // You can dynamically tune FF you do the same thing I did with PID where I created a new class
  // for it though you wouldn't want to as FF should be calculated empirically if possible
  private SimpleMotorFeedforward intakeSimpleFF =
      new SimpleMotorFeedforward(0.218079, 0.120429, 0.0);

  private TunablePIDController intakePivotPID =
      new TunablePIDController(0.3, 0.0, 0.0, "/intakePivot/PivotPID/");

  /** Creates a new Intake. */
  public Intake() {
    PhoenixUtil.configMotor(intake, false, NeutralModeValue.Coast);
    PhoenixUtil.configMotor(pivot, true, NeutralModeValue.Brake);
  }

  /** Function for voltage control for intake motor */
  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  /** Function for voltage control for pivot motor */
  public void setPivotVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  /** Returns the amount of rotations of the pivot motor */
  public double getPivotPosition() {
    return pivot.getPosition().getValueAsDouble();
  }

  /** Returns intake velocity in units of RPS */
  public double getIntakeVelocity() {
    return intake.getVelocity().getValueAsDouble();
  }

  /**
   * Sets the Intake to a target velocity and should be called periodically unit are in revolution
   * per second
   */
  public void setIntakeRPS(double RPS) {
    Logger.recordOutput("Intake/TargetVelocity", RPS);
    double pidPart = intakePID.calculate(getIntakeVelocity(), RPS);
    double ffPart = intakeSimpleFF.calculateWithVelocities(getIntakeVelocity(), RPS);
    intake.setVoltage(pidPart + ffPart);
  }

  /** Sets the pivot to extended state and should be called periodically unit are in revolutions */
  public void extendIntake() {
    pivot.setVoltage(intakePivotPID.calculate(getPivotPosition(), Constants.INTAKE_EXTENDED));
  }

  /** Sets the pivot to retracted state and should be called periodically unit are in revolutions */
  public void retractIntake() {
    pivot.setVoltage(intakePivotPID.calculate(getPivotPosition(), Constants.INTAKE_RETRACTED));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake/PivotPosition", getPivotPosition());
    Logger.recordOutput("Intake/Velocity", getIntakeVelocity());

    intakePivotPID.update();
  }
}
