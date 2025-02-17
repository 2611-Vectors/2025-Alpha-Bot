// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final TalonFX intake = new TalonFX(Constants.INTAKE_MOTOR_ID);
  private final TalonFX pivot = new TalonFX(Constants.PIVOT_MOTOR_ID);
  private final TalonFX endEffector = new TalonFX(43);
  private TunablePIDController intakePIDController =
      new TunablePIDController(0.0, 0.0, 0.0, "/Intake/IntakePID/");
  // Can not dynamiclly tune feed forward as far as we know
  private SimpleMotorFeedforward intakeSimpleFFController =
      new SimpleMotorFeedforward(0.218079, 0.120429, 0.0);

  private TunablePIDController intakePivotPIDController =
      new TunablePIDController(0.3, 0.0, 0.0, "/intakePivot/PivotPID/");
  private ArmFeedforward intakePivotFeedforwardController = new ArmFeedforward(0.0, 0.0, 0.0);

  private double intakeRPS = 0.0;

  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0);

  /** Creates a new Intake. */
  public Intake() {
    PhoenixUtil.configMotor(intake, false, intakePIDController, intakePivotFeedforwardController);

    PhoenixUtil.configMotor(
        pivot,
        true,
        new PIDController(0, 0, 0),
        new ArmFeedforward(0, 0, 0),
        NeutralModeValue.Brake);
  }

  public void setEndEffectorVoltage(double voltage) {
    endEffector.setVoltage(voltage);
  }

  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public void setPivotVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  public double getPivotPosition() {
    return pivot.getPosition().getValueAsDouble();
  }

  public PIDController getIntakePivotPID() {
    return intakePivotPIDController;
  }

  public void setIntakePivotPID(double p, double i, double d) {
    intakePivotPIDController.setP(p);
    intakePivotPIDController.setI(i);
    intakePivotPIDController.setD(d);
  }

  public void setIntakePID(double p, double i, double d) {
    intakePIDController.setP(p);
    intakePIDController.setI(i);
    intakePIDController.setD(d);
  }

  public void setIntakeFF(double s, double v, double a) {
    intakeSimpleFFController = new SimpleMotorFeedforward(s, v, a);
  }

  public void setIntakeRPS(double RPS) {
    Logger.recordOutput("Intake/TargetVelocity", RPS);
    intake.setVoltage(
        intakePIDController.calculate(intake.getVelocity().getValueAsDouble(), RPS)
            + intakeSimpleFFController.calculateWithVelocities(
                intake.getVelocity().getValueAsDouble(), RPS));
  }

  public void extendIntake() {
    pivot.setVoltage(intakePivotPIDController.calculate(getPivotPosition(), 15.0));
  }

  public void retractIntake() {
    pivot.setVoltage(intakePivotPIDController.calculate(getPivotPosition(), 1.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake/EncoderPosition", getPivotPosition());
    Logger.recordOutput("Intake/Velocity", intake.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/PivotPosition", pivot.getPosition().getValueAsDouble());
    // intake.setVoltage(
    // intakePIDController.calculate(intake.getVelocity().getValueAsDouble(),
    // intakeRPS)
    // + intakeSimpleFFController.calculate(intakeRPS));
    // currentIntakeRPS.set(intake.getVelocity().getValueAsDouble());
    // currentIntakeVoltage.set(intake.getMotorVoltage().getValueAsDouble());
    intakePivotPIDController.update();
  }
}
