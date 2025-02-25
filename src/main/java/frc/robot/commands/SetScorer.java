// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;

/** Add your docs here. */
public class SetScorer {
  public static Command set(Elevator m_Elevator, Arm m_Arm, double height, double angle) {
    return Commands.run(
            () -> {
              m_Elevator.setElevatorPosition(height);
              m_Arm.setPivotAngle(angle);
            },
            m_Elevator,
            m_Arm)
        .finallyDo(
            () -> {
              m_Elevator.setVoltage(m_Elevator.elevatorFF.getKg());
              m_Arm.setArmVoltage(0.0);
            });
  }
}
