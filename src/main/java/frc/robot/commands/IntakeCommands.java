// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Intake;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class IntakeCommands {
  /** Command to set the voltage based off the tunable parameters in Advantage Scope */
  public static Command IntakeTestCommands(Intake m_intake) {
    LoggedNetworkNumber intakeVoltage = new LoggedNetworkNumber("/Intake/IntakeVoltage", 0.0);
    LoggedNetworkNumber pivotVoltage = new LoggedNetworkNumber("/Intake/PivotVoltage", 0.0);

    return Commands.run(
        () -> {
          m_intake.setIntakeVoltage(intakeVoltage.get());
          m_intake.setPivotVoltage(pivotVoltage.get());
        },
        m_intake);
  }

  public static Command IntakeRPSTestCommands(Intake m_intake) {
    LoggedNetworkNumber intakeRPS = new LoggedNetworkNumber("/Testing/IntakeRPS", 0.0);

    return Commands.run(
        () -> {
          m_intake.setIntakeRPS(intakeRPS.get());
        },
        m_intake);
  }

  /** Voltage control of intake and pivot based off controllers */
  public static Command IntakeSimpleController(
      Intake m_intake, Supplier<Double> intakeSupplier, Supplier<Double> pivotSupplier) {
    return Commands.run(
        () -> {
          m_intake.setIntakeVoltage(intakeSupplier.get() * 8);
          m_intake.setPivotVoltage(pivotSupplier.get() * 8);
        },
        m_intake);
  }
}
