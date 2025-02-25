// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Arm;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import static frc.robot.Constants.Setpoints.*;

/** Add your docs here. */
public class ArmCommands {
  public static Command ArmTestCommand(Arm m_Arm) {
    LoggedNetworkNumber pivotAngle = new LoggedNetworkNumber("/Testing/PivotAngle", -90);
    return Commands.run(
        () -> {
          m_Arm.setPivotAngle(pivotAngle.get());
        },
        m_Arm);
  }

  public static Command ArmSimpleController(
      Arm m_Arm, Supplier<Double> armSupplier, Supplier<Double> endEffectorSupplier) {
    return Commands.run(
        () -> {
          m_Arm.setArmVoltage(armSupplier.get() * 4);
        },
        m_Arm);
  }

  public static Command EndEffectorController(Arm m_Arm, Supplier<Double> endEffectorSupplier) {
    return Commands.run(() -> m_Arm.setEndEffectorVoltage(endEffectorSupplier.get() * 4), m_Arm);
  }

  public static Command waitUntilArmAngle(Arm m_Arm, double angle) {
    return Commands.waitUntil(() -> Math.abs(Arm.getRelativeAngle(angle, m_Arm.getPivotAngle())) < ANGLE_TOLERANCE);
  }
}
