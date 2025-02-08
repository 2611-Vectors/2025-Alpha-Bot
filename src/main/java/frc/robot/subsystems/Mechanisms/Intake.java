// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final TalonFX intake = new TalonFX(Constants.INTAKE_MOTOR_ID);
  private final TalonFX pivot = new TalonFX(Constants.PIVOT_MOTOR_ID);

  private PIDController intakePivotPIDController = new PIDController(0.0, 0.0, 0.0);
  private ArmFeedforward intakePivotFeedforwardController = new ArmFeedforward(0.0, 0.0, 0.0);

  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0);

  /** Creates a new Intake. */
  public Intake() {
    PhoenixUtil.configMotors(
        intake, false, intakePivotPIDController, intakePivotFeedforwardController);
  }

  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
  }

  public void setPivotVoltage(double volatge) {
    pivot.setVoltage(volatge);
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

  public void extendIntake() {
    pivot.setControl(m_PositionVoltage.withPosition(10));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake/EncoderPosition", getPivotPosition());
  }
}
