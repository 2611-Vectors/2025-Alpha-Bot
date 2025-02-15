// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// Feed
public class Transition extends SubsystemBase {
  public DigitalInput irSenser = new DigitalInput(1);
  private SparkFlex transitionMotor;
  TunablePIDController controllerPID;

  private SimpleMotorFeedforward transitionSimpleFFController;

  LoggedNetworkNumber currentTransitionRPS, currentTransitionVoltage;

  /** Creates a new Transition. */
  public Transition() {
    transitionMotor = new SparkFlex(Constants.TRANSITION_ID, MotorType.kBrushless);
    transitionSimpleFFController = new SimpleMotorFeedforward(0.45661, 0.11467, 0.0);
    currentTransitionRPS = new LoggedNetworkNumber("/Intake/TransitionRPS", 0.0);
    currentTransitionVoltage = new LoggedNetworkNumber("/Intake/TransitionVoltage", 0.0);

    controllerPID =
        new TunablePIDController(
            Constants.TRANSITION_P,
            Constants.TRANSITION_I,
            Constants.TRANSITION_D,
            "/Intake/TransitionPID/");
  }

  public void setPower(double power) {
    transitionMotor.set(power);
  }

  public void setVoltage(double voltage) {
    transitionMotor.setVoltage(voltage);
  }

  public void setTransitionRPS(double RPS) {
    Logger.recordOutput("/Transition/TargetVelocity", RPS);
    transitionMotor.setVoltage(
        controllerPID.calculate(transitionMotor.getEncoder().getVelocity(), RPS)
            + transitionSimpleFFController.calculateWithVelocities(
                transitionMotor.getEncoder().getVelocity(), RPS));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // transitionMotor.setVoltage(
    //     controllerPID.calculate(transitionMotor.getEncoder().getVelocity(), transitionRPS)
    //         + transitionSimpleFFController.calculate(transitionRPS));
    // currentTransitionRPS.set(transitionMotor.getEncoder().getVelocity());
    // // transitionMotor.getBusVoltage() * transitionMotor.getAppliedOutput() should output applied
    // // voltage
    // currentTransitionVoltage.set(
    //     transitionMotor.getBusVoltage() * transitionMotor.getAppliedOutput());
    Logger.recordOutput(
        "Transition/VelocityRPS", transitionMotor.getEncoder().getVelocity() / 60.0);
  }
}
