// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private final SparkMax winchMotor;

  private final SparkMax grabMotor;

  public Climb() {
    winchMotor = new SparkMax(Constants.CLIMB_WINCH_ID, MotorType.kBrushless);
    grabMotor = new SparkMax(Constants.CLIMB_GRAB_ID, MotorType.kBrushless);
  }

  public void setWinchVoltage(double voltage) {
    winchMotor.setVoltage(voltage);
  }

  public void setGrabVoltage(double voltage) {
    grabMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {}
}
