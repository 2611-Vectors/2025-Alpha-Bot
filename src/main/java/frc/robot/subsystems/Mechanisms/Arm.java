// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final TalonFX arm = new TalonFX(Constants.ARM_MOTOR_ID);
  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.ARM_PIVOT_PORT);
  TunablePIDController armPID = new TunablePIDController(0.1, 0.0, 0.0, "/Testing/ArmPID/");

  /** Creates a new Arm. */
  public Arm() {
    // Motor Voltage and Speed
    PhoenixUtil.configMotor(
        arm, true, new PIDController(0, 0, 0), new ArmFeedforward(0, 0, 0), NeutralModeValue.Brake);
    arm.setPosition(0);
    armPID.enableContinuousInput(-180, 180);
  }

  // ^change later

  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  public double getPivotAngle() {
    // Clockwise should result in positive encoder values
    double rotations = arm.getPosition().getValueAsDouble() / Constants.ARM_GEAR_RATIO;
    double scaledRotation = rotations % 1;
    // Range is -180 - 180
    double degrees = (scaledRotation - 0.5) * 360 - Constants.PIVOT_ANGLE_OFFSET;
    if (degrees < -180) {
      degrees += 360;
    }
    if (degrees > 180) {
      degrees -= 360;
    }
    return degrees;

    // This code works just our absolute encoder doesnt
    // double correctedAngle = (pivotEncoder.get() * 360 + Constants.PIVOT_ANGLE_OFFSET) % 360;
    // if (correctedAngle < 0) {
    //   correctedAngle += 360;
    // }
    // if (correctedAngle > 360) {
    //   correctedAngle %= 360;
    // }
    // return correctedAngle;
  }

  public static double getRelativeAngle(double angle1, double angle2) {
    double difference = (angle2 - angle1 + 180) % 360 - 180;
    Logger.recordOutput(
        "Testing/AngleDifference", difference < -180 ? difference + 360 : difference);
    return difference < -180 ? difference + 360 : difference;
  }

  public void setPivotAngle(double angle) {
    double pidPart = armPID.calculate(getPivotAngle(), angle);
    double ffPart = 0;
    arm.setVoltage(
        MathUtil.clamp(pidPart + ffPart, -Constants.ARM_MAX_VOLTAGE, Constants.ARM_MAX_VOLTAGE));
    Logger.recordOutput("Arm/TargetAngle", angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm/current angle", getPivotAngle());
    armPID.update();
  }
}
