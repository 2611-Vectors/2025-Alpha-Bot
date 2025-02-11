// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Transition;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class TransitionCommands {
  public static Command TransitionTestCommand(Transition m_transition) {
    LoggedNetworkNumber transitionVoltage =
        new LoggedNetworkNumber("/transition/transitionVoltage/", 0.0);
    return Commands.run(
        () -> {
          m_transition.setVoltage(transitionVoltage.get());
        },
        m_transition);
  }

  public static Command SimpleTransitionController(
      Transition m_transition, Supplier<Double> transitionSupplier) {

    return Commands.run(
        () -> {
          m_transition.setVoltage(transitionSupplier.get() * 8);
        },
        m_transition);
  }
}
