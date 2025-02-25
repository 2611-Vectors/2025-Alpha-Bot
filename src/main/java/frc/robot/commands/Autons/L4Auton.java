// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.util.CustomAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4Auton extends SequentialCommandGroup {

  /** Creates a new L4Auton. */
  public L4Auton(Elevator m_Elevator, Arm m_Arm) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();
    Command autoCommand = drivePaths[0];
    addCommands(autoCommand); // , new ScoreSetpoints(m_Elevator, m_Arm, L4_HEIGHT_IN, L4_ANGLE));
  }
}
