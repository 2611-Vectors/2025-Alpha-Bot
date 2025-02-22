// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Home extends SequentialCommandGroup {
  /** Creates a new Home. */
  public Home(Elevator m_Elevator, Arm m_Arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.race(
            Commands.sequence(
                Commands.run(() -> m_Arm.setPivotAngle(HOME_ANGLE)),
                Commands.run(() -> m_Elevator.setVoltage(m_Elevator.elevatorFF.getKg()))),
            Commands.waitUntil(
                () ->
                    Math.abs(Arm.getRelativeAngle(HOME_ANGLE, m_Arm.getPivotAngle()))
                        < ANGLE_TOLERANCE)),
        Commands.race(
            SetScorer.set(m_Elevator, m_Arm, HOME_HEIGHT_IN, HOME_ANGLE),
            Commands.waitUntil(
                () ->
                    Math.abs(HOME_HEIGHT_IN - m_Elevator.getLeftElevatorPosition())
                        < POSITION_TOLERANCE)));
  }
}
