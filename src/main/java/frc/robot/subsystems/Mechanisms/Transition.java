// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunablePIDController;

// Feed
public class Transition extends SubsystemBase {
  public DigitalInput irSenser = new DigitalInput(1);
  private SparkFlex transitionMotor;
  TunablePIDController controllerPID;

  /** Creates a new Transition. */
  public Transition() {
    transitionMotor = new SparkFlex(Constants.TRANSITION_ID, MotorType.kBrushless);

    controllerPID =
        new TunablePIDController(
            Constants.TransitionP,
            Constants.TransitionI,
            Constants.TransitionD,
            "/tunning/transition/");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(controllerPID.getP() != oldP || controllerPID.getI() != oldI || controllerPID.getD() !=
    // oldD){
    //   configMotors(transitionMotor, false);
    // }
  }

  public void setPower(double power) {
    transitionMotor.set(power);
  }

  public void setVoltage(double voltage) {
    transitionMotor.setVoltage(voltage);
  }
}
