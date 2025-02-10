// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  TunablePIDController controllerPID;
  private final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0);
  private double oldP = 0.0, oldI = 0.0, oldD = 0.0;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(Constants.LEFT_ELEVATOR_ID);
    rightMotor = new TalonFX(Constants.RIGHT_ELEVATOR_ID);

    PhoenixUtil.configMotors(leftMotor, false, controllerPID, new ArmFeedforward(0.0, 0.0, 0.0));
    PhoenixUtil.configMotors(rightMotor, true, controllerPID, new ArmFeedforward(0.0, 0.0, 0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (controllerPID.getP() != oldP
        || controllerPID.getI() != oldI
        || controllerPID.getD() != oldD) {
      PhoenixUtil.configMotors(leftMotor, false, controllerPID, new ArmFeedforward(0.0, 0.0, 0.0));
      PhoenixUtil.configMotors(rightMotor, true, controllerPID, new ArmFeedforward(0.0, 0.0, 0.0));
    }

    controllerPID =
        new TunablePIDController(
            Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D, "/tunning/elevator/");
  }

  public void setPower(double power) {
    leftMotor.set(power);
    rightMotor.set(power);
  }

  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void setElevatorPosition(double motorPosition) {
    Logger.recordOutput("Elevator/Position", motorPosition);

    // Add a simulator update

    leftMotor.setControl(m_PositionVoltage.withPosition(motorPosition));
    rightMotor.setControl(m_PositionVoltage.withPosition(motorPosition));
  }

  public double getLeftElevatorPosition() {
    return leftMotor.getPosition().getValueAsDouble() * Constants.ROTATIONS_TO_INCHES;
  }

  public double getRightElevatorPosition() {
    return rightMotor.getPosition().getValueAsDouble() * Constants.ROTATIONS_TO_INCHES;
  }
}
