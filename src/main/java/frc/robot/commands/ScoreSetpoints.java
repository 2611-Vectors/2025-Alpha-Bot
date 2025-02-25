// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreSetpoints extends SequentialCommandGroup {
  /** Creates a new ScoreSetpoints. */
  public ScoreSetpoints(Elevator m_Elevator, Arm m_Arm, double height, double angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.race(
            SetScorer.set(m_Elevator, m_Arm, height, HOME_ANGLE),
            Commands.waitUntil(
                () ->
                    Math.abs(height - m_Elevator.getLeftElevatorPosition()) < POSITION_TOLERANCE)),
        Commands.race(
            Commands.run(() -> m_Arm.setEndEffectorVoltage(1)),
            SetScorer.set(m_Elevator, m_Arm, height, Arm.flipAngle(angle)),
            Commands.waitUntil(
                () ->
                    Math.abs(Arm.getRelativeAngle(Arm.flipAngle(angle), m_Arm.getPivotAngle()))
                        < ANGLE_TOLERANCE)),
        Commands.race(
            SetScorer.set(m_Elevator, m_Arm, height, Arm.flipAngle(angle)),
            Commands.run(() -> m_Arm.setEndEffectorVoltage(-2)),
            new WaitCommand(2)),
        new Home(m_Elevator, m_Arm));
  }
}
