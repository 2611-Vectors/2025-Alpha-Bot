// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Elevator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class ElevatorCommands {
  public static Command ElevatorTestCommand(Elevator m_Elevator) {
    LoggedNetworkNumber elevatorPosition = new LoggedNetworkNumber("/Testing/ElevatorPosition", 0.0);
    return m_Elevator.setElevatorPosition(() -> elevatorPosition.get());
  }

  public static Command ElevatorVoltageControl(Elevator m_Elevator, Supplier<Double> voltage) {
    LoggedNetworkNumber voltageTuning = new LoggedNetworkNumber("/Elevator/Voltage", 0.0);
    return Commands.run(
        () -> {
          m_Elevator.setVoltage(voltage.get() * 4 + voltageTuning.get());
        },
        m_Elevator);
  }

  public static Command ElevatorFFTuner(Elevator m_Elevator) {
    LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Elevator/Voltage", 0.0);
    return Commands.run(
        () -> {
          m_Elevator.setVoltage(voltage.get());
        },
        m_Elevator);
  }
}
