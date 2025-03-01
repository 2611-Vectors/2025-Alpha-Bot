// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.util.CustomAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Left3Auton extends SequentialCommandGroup {

  /** Creates a new L4Auton. */
  public Left3Auton(Elevator m_Elevator, Arm m_Arm) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();

    for (Command segment : drivePaths) {
      addCommands(segment, Commands.waitSeconds(1.0));
    }

    // addCommands(
    //     autoCommand,
    //     Commands.waitSeconds(1.5),
    //     // new ScoreSetpoints(m_Elevator, m_Arm, L3_HEIGHT_IN, L3_ANGLE), // DONT MOVE THE
    // ELEVATOR
    //     // OR ARM
    //     Commands.parallel(
    //         drivePaths[1],
    //         Commands.sequence(new WaitCommand(2), new LoadStationIntake(m_Elevator, m_Arm))));
  }
}
