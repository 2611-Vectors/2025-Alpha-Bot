// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(Constants.LEFT_ELEVATOR_ID);
    rightMotor = new TalonFX(Constants.RIGHT_ELEVATOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    configMotors(leftMotor, false);
    configMotors(rightMotor, false);

    controllerPID =
        new TunablePIDController(
            Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D, "/tunning/elevator");
  }

  public void configMotors(TalonFX motor, boolean inverted) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 5; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    configs.Slot0.kS = 0; // Baseline voltage required to overcome static forces like friction
    configs.Slot0.kG = 0; // Voltage to overcome gravity
    configs.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // Peak output of 8 V
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    configs.CurrentLimits.StatorCurrentLimit = 60;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // Example on how you would do break mode / coast mode
    // configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(configs, 0.25));
    motor.setPosition(0);
  }

  public void setPower(double power) {
    leftMotor.set(power);
    rightMotor.set(power);
  }

  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void setElevatorPosition(double leftPosition, double rightPosition) {
    Logger.recordOutput("Elevator/Position", leftPosition);
    Logger.recordOutput("Elevator/Position", rightPosition);

    // Add a simulator update

    leftMotor.setControl(m_PositionVoltage.withPosition(leftPosition));
    rightMotor.setControl(m_PositionVoltage.withPosition(rightPosition));
  }

  public double getLeftElevatorPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  public double getRightElevatorPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }
}
