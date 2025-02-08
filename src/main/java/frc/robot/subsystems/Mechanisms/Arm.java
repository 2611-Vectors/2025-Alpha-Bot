// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final TalonFX arm = new TalonFX(Constants.ARM_MOTOR_ID);
  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.ARM_PIVOT_PORT);
  // PIDController ControllerPID
  public void setMotor() {
    // Motor Voltage and Speed
    PhoenixUtil.configMotors(arm, false, new PIDController(0, 0, 0), new ArmFeedforward(0, 0, 0));
  }

  /** Creates a new Arm. */
  public Arm() {}

  // ^change later

  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  public double getPivotAngle() {
    double correctedAngle = pivotEncoder.getFrequency() * 360 + Constants.PIVOT_ANGLE_OFFSET;
    if (correctedAngle < 0) {
      correctedAngle += 360;
    }
    if (correctedAngle > 360) {
      correctedAngle %= 360;
    }
    return correctedAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm/current angle", getPivotAngle());
  }
}
