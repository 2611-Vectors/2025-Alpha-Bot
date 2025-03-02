// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ActionHandlers;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;

public class ScoringScheduler extends SubsystemBase {
  /** Creates a new ScoringHandler. */
  private final Elevator m_Elevator;

  private final Arm m_Arm;

  public ScoringScheduler(Elevator m_Elevator, Arm m_Arm) {
    this.m_Elevator = m_Elevator;
    this.m_Arm = m_Arm;
  }

  @Override
  public void periodic() {}

  public Command setScorer(double height, double angle) {
    return Commands.sequence(
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(angle)),
            ArmCommands.waitUntilArmAngle(m_Arm, angle)),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(angle)),
            Commands.run(() -> m_Elevator.setElevatorPosition(height)),
            ElevatorCommands.waitUntilElevatorHeight(m_Elevator, height)),
        Commands.runOnce(
            () -> {
              m_Elevator.setVoltage(m_Elevator.elevatorFF.getKg());
              m_Arm.setArmVoltage(0.0); // GET A FEED FORWARD VALUE!!!
            }));
  }

  public Command scoreSetpoint(double height, double angle) {
    Command ret =
        Commands.sequence(
        setScorer(height, HOME_ANGLE),
        Commands.runOnce(() -> m_Arm.setEndEffectorVoltage(0)),
        setScorer(height, Arm.flipAngle(angle)),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(Arm.flipAngle(angle))),
            Commands.run(() -> m_Arm.setEndEffectorVoltage(2)),
            new WaitCommand(2)),
        Commands.runOnce(() -> m_Arm.setEndEffectorVoltage(0)),
        Commands.run(() -> m_Arm.setPivotAngle(Arm.flipAngle(angle))));

    return runEnd(
        () -> ret.schedule(),
        () -> {
          ret.cancel();
          m_Arm.setEndEffectorVoltage(0);
        });
  }

  public Command autoLoadStationIntake() {
    return Commands.sequence(
        setScorer(INTAKE_HEIGHT_IN, HOME_ANGLE),
        setScorer(INTAKE_HEIGHT_IN, INTAKE_ANGLE),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(INTAKE_ANGLE)),
            Commands.run(() -> m_Arm.setEndEffectorVoltage(-2)),
            Commands.waitUntil(() -> Math.abs(m_Arm.getEndEffectorRPS()) > 9.0)),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(INTAKE_ANGLE)),
            Commands.waitUntil(() -> Math.abs(m_Arm.getEndEffectorRPS()) < 8.0)),
        Commands.runOnce(() -> m_Arm.setEndEffectorVoltage(0)));
  }

  public Command autoTravelPosition() {
    return setScorer(TRAVEL_HEIGHT, TRAVEL_ANGLE);
  }

  public Command holdArmPos(double angle) {
    return run(() -> m_Arm.setPivotAngle(Arm.flipAngle(angle)));
  }

  public Command holdPosition(double height, double angle) {
    return run(() -> {
      m_Arm.setPivotAngle(Arm.flipAngle(angle));
      m_Elevator.setElevatorPosition(height);
    });
  }

  public Command autoScoreSetpoint(double height, double angle) {
    return Commands.sequence(
        setScorer(height, HOME_ANGLE),
        setScorer(height, Arm.flipAngle(angle)),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(Arm.flipAngle(angle))),
            Commands.run(() -> m_Arm.setEndEffectorVoltage(2)),
            new WaitCommand(2)),
        Commands.runOnce(() -> m_Arm.setEndEffectorVoltage(0)));
  }

  public Command loadStationIntake() {
    Command ret =
        Commands.sequence(
        setScorer(INTAKE_HEIGHT_IN, HOME_ANGLE),
        setScorer(INTAKE_HEIGHT_IN, INTAKE_ANGLE),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(INTAKE_ANGLE)),
            Commands.run(() -> m_Arm.setEndEffectorVoltage(-2)),
            Commands.waitUntil(() -> Math.abs(m_Arm.getEndEffectorRPS()) > 9.0)),
        Commands.race(
            Commands.run(() -> m_Arm.setPivotAngle(INTAKE_ANGLE)),
            Commands.waitUntil(() -> Math.abs(m_Arm.getEndEffectorRPS()) < 8.0)),
        Commands.run(() -> m_Arm.setEndEffectorVoltage(0)));

    return runEnd(
        () -> ret.schedule(),
        () -> {
          ret.cancel();
          m_Arm.setEndEffectorVoltage(0);
        });
  }

  public Command travelPosition() {
    Command ret = setScorer(TRAVEL_HEIGHT, TRAVEL_ANGLE);
    return runEnd(() -> ret.schedule(), () -> ret.cancel());
  }
}
