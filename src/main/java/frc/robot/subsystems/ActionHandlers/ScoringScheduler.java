// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ActionHandlers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;

import static frc.robot.Constants.Setpoints.*;

public class ScoringScheduler extends SubsystemBase {
    /** Creates a new ScoringHandler. */

    private final Elevator m_Elevator;
    private final Arm m_Arm;

    public ScoringScheduler(Elevator m_Elevator, Arm m_Arm) {
        this.m_Elevator = m_Elevator;
        this.m_Arm = m_Arm;
    }

    @Override public void periodic() {}

    public Command setScorer(double height, double angle) {
        return Commands.sequence(
            Commands.race(
                Commands.run(() -> m_Arm.setPivotAngle(angle)),
                ArmCommands.waitUntilArmAngle(m_Arm, angle)),
            Commands.race(
                Commands.run(() -> m_Elevator.setElevatorPosition(height)),
                ElevatorCommands.waitUntilElevatorHeight(m_Elevator, height)),
            Commands.run(() -> {
                m_Elevator.setVoltage(m_Elevator.elevatorFF.getKg());
                m_Arm.setArmVoltage(0.0); // GET A FEED FORWARD VALUE!!!
            }));
    }

    public Command scoreSetpoint(double height, double angle) {
        return Commands.sequence(
            setScorer(height, HOME_ANGLE),
            Commands.run(() -> m_Arm.setEndEffectorVoltage(1)),
            setScorer(height, Arm.flipAngle(angle)),
            Commands.race(
                setScorer(height, Arm.flipAngle(angle)),
                Commands.run(() -> m_Arm.setEndEffectorVoltage(-2)),
                new WaitCommand(2)));
    }

    public Command loadStationIntake() {
        return Commands.sequence(
            setScorer(INTAKE_HEIGHT_IN, HOME_ANGLE),
            setScorer(INTAKE_HEIGHT_IN, INTAKE_ANGLE),
            Commands.race(
                Commands.run(() -> m_Arm.setEndEffectorVoltage(2)),
                Commands.waitUntil(() -> Math.abs(m_Arm.getEndEffectorRPS()) > 13.0)),
            Commands.race(
                Commands.run(() -> m_Arm.setEndEffectorVoltage(2)),
                Commands.waitUntil(() -> Math.abs(m_Arm.getEndEffectorRPS()) < 13.0)));
    }

    public Command travelPosition() {
        return setScorer(TRAVEL_HEIGHT, TRAVEL_ANGLE);
    }
}
