// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MechanismSimulator;
import frc.robot.util.MechanismSimulatorActual;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;

import static frc.robot.Constants.Setpoints.HOME_ANGLE;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final TalonFX arm = new TalonFX(Constants.ARM_MOTOR_ID);
  private final TalonFX endEffector = new TalonFX(Constants.END_EFFECTOR_ID);

  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.ARM_PIVOT_PORT);
  TunablePIDController armPID = new TunablePIDController(0.1, 0.0, 0.0, "/Testing/ArmPID/");

  /** Creates a new Arm. */
  public Arm() {
    PhoenixUtil.configMotor(arm, true, NeutralModeValue.Brake);
    PhoenixUtil.configMotor(endEffector, false, NeutralModeValue.Coast);
    armPID.enableContinuousInput(-180, 180);
  }

  /** Function for voltage control for arm motor */
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  public Command setEndEffectorVoltage(Supplier<Double> voltage) {
    return runOnce(() -> endEffector.setVoltage(voltage.get()));
  }

  /**
   * Returns arm angle in units of degrees from -180 to 180 see
   * https://natedcoder.github.io/Arm-Simulation/ for visualiion
   */
  public double getPivotAngle() {
    // Using relative since the absolute encoder doesnt work
    // Clockwise should result in positive encoder values
    double rotations = arm.getPosition().getValueAsDouble() / Constants.ARM_GEAR_RATIO;
    double scaledRotation = rotations % 1;
    // Range is -180 to 180
    double degrees = (scaledRotation - 0.5) * 360 - Constants.PIVOT_ANGLE_OFFSET;
    if (degrees < -180) {
      degrees += 360;
    }
    if (degrees > 180) {
      degrees -= 360;
    }
    return degrees;
  }

  /** Function to calculate the relative distance between two angles */
  public static double getRelativeAngle(double angle1, double angle2) {
    double difference = (angle2 - angle1 + 180) % 360 - 180;
    return difference < -180 ? difference + 360 : difference;
  }

  /** Function to flip the angle for other side of the robot */
  public static double flipAngle(double angle) {
    double reflectedAngle = -180 - angle;
    if (reflectedAngle < -180) {
      return reflectedAngle + 360;
    }
    return reflectedAngle;
  }

  /**
   * Sets the Arm Pivot to a target position and should be called periodically unit are in degrees
   */
  public Command setPivotAngle(Supplier<Double> angle) {
    return run(() -> {
      Logger.recordOutput("Arm/TargetAngle", angle.get());
      MechanismSimulator.updateArm(angle.get());

      double pidPart;
      if (!MechanismSimulator.isLegalTarget()) {
        pidPart = armPID.calculate(getPivotAngle(), HOME_ANGLE);
      } else {
        pidPart = armPID.calculate(getPivotAngle(), angle.get());
      }
      
      double ffPart = 0;
      arm.setVoltage(
          MathUtil.clamp(pidPart + ffPart, -Constants.ARM_MAX_VOLTAGE, Constants.ARM_MAX_VOLTAGE));
    });
  }

  public double getEndEffectorRPS() {
    return endEffector.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm/current angle", getPivotAngle());
    Logger.recordOutput("Arm/End Effector RPS", getEndEffectorRPS());
    MechanismSimulatorActual.updateArm(getPivotAngle());
    armPID.update();
  }
}
