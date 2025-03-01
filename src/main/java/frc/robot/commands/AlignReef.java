// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AutonConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.TunablePIDController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignReef extends SequentialCommandGroup {
  /** Creates a new alignReef. */
  TunablePIDController drivePID_X = new TunablePIDController(1, 0, 0, "");

  TunablePIDController drivePID_Y = new TunablePIDController(1, 0, 0, "");

  private Pose2d getClosestPoint(Pose2d currentPos) {
    Pose2d closestPoint = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    for (Pose2d position : poseAngleMap.keySet()) {
      double dist = position.getTranslation().getDistance(currentPos.getTranslation());
      if (dist < closestDistance) {
        closestDistance = dist;
        closestPoint = position;
      }
    }

    return closestPoint;
  }

  public AlignReef(Drive m_Drive, double reefSide) {
    Pose2d closestPoint = getClosestPoint(m_Drive.getPose());
    Pose2d targetPos = CustomAutoBuilder.applyOffset(closestPoint, reefSide);

    addCommands(
        Commands.parallel(
            Commands.run(
                () -> {
                  drivePID_X.update();
                  drivePID_Y.update();
                }),
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () -> drivePID_X.calculate(targetPos.getX(), m_Drive.getPose().getX()),
                () -> drivePID_Y.calculate(targetPos.getY(), m_Drive.getPose().getY()),
                () -> Rotation2d.fromDegrees(poseAngleMap.get(closestPoint)))));
  }
}
