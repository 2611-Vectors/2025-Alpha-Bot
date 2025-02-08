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
    LoggedNetworkNumber elevatorPosition =
        new LoggedNetworkNumber("/Elevator/ElevatorPosition", 0.0);
    return Commands.run(
        () -> {
          m_Elevator.setElevatorPosition(elevatorPosition.get());
        },
        m_Elevator);
  }

  public static Command ElevatorSimpleController(
      Elevator m_Elevator,
      Supplier<Boolean> elevatorSupplierA,
      Supplier<Boolean> elevatorSupplierB,
      Supplier<Boolean> elevatorSupplierX,
      Supplier<Boolean> elevatorSupplierY) {

    return Commands.run(
        () -> {
          if (elevatorSupplierA.get()) {
            m_Elevator.setElevatorPosition(0.0);
          } else if (elevatorSupplierB.get()) {
            m_Elevator.setElevatorPosition(1.0);
          } else if (elevatorSupplierX.get()) {
            m_Elevator.setElevatorPosition(2.0);
          } else if (elevatorSupplierY.get()) {
            m_Elevator.setElevatorPosition(3.0);
          }
        },
        m_Elevator);
  }
}
