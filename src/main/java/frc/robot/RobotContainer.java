// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.Setpoints.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CustomAutoBuilder;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  //   private final Intake m_Intake;
  //   private final Transition m_Transition;
  //   private final Elevator m_Elevator;
  //   private final Arm m_Arm;

  private final Vision m_Vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        m_Vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.reefCamName, VisionConstants.robotToReefCam),
                new VisionIOPhotonVision(
                    VisionConstants.loadStationCamName, VisionConstants.robotToLoadStationCam));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        m_Vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.reefCamName, VisionConstants.robotToReefCam, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.loadStationCamName,
                    VisionConstants.robotToLoadStationCam,
                    drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_Vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // m_Intake = new Intake();
    // m_Transition = new Transition();
    // m_Elevator = new Elevator();
    // m_Arm = new Arm();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));

    // m_Intake.setDefaultCommand(
    //     IntakeCommands.IntakeRPSTestCommands(
    //         m_Intake)); // , () -> operatorController.getLeftY(), () -> 0.0));
    // m_Arm.setDefaultCommand(
    //     ArmCommands.EndEffectorController(m_Arm, () -> operatorController.getLeftY()));
    // m_Intake.setDefaultCommand(
    // IntakeCommands.IntakeSimpleController(
    // m_Intake, () -> operatorController.getLeftY(), () ->
    // operatorController.getRightY()));
    // m_Transition.setDefaultCommand(
    //     TransitionCommands.TransitionRPSTestCommand(
    //         m_Transition)); // , () -> operatorController.getRightY()));

    // operatorController
    //     .a()
    //     .onTrue(
    //         new ScoreSetpoints(m_Elevator, m_Arm, L2_HEIGHT_IN, L2_ANGLE)
    //             .andThen(SetScorer.set(m_Elevator, m_Arm, HOME_HEIGHT_IN, HOME_ANGLE)));

    // operatorController.b().whileTrue(new ScoreSetpoints(m_Elevator, m_Arm, L3_HEIGHT_IN,
    // L3_ANGLE));
    // operatorController.y().whileTrue(new ScoreSetpoints(m_Elevator, m_Arm, L4_HEIGHT_IN,
    // L4_ANGLE));

    // operatorController.x().whileTrue(new Home(m_Elevator, m_Arm));
    // operatorController.start().whileTrue(new LoadStationIntake(m_Elevator, m_Arm));

    // operatorController
    //     .rightBumper()
    //     .whileTrue(
    //         Commands.run(() -> m_Intake.extendIntake())
    //             .finallyDo(() -> m_Intake.setPivotVoltage(0.0)));
    // operatorController
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.run(() -> m_Intake.retractIntake())
    //             .finallyDo(() -> m_Intake.setPivotVoltage(0.0)));
    // m_Elevator.setDefaultCommand(
    // ElevatorCommands.ElevatorTestCommand(
    // m_Elevator)); // , () -> -operatorController.getLeftY()));
    // m_Arm.setDefaultCommand(
    // ArmCommands.ArmTestCommand(m_Arm)); // , () -> operatorController.getLeftY(),
    // () ->
    // operatorController.getRightY()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Back button is pressed
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drive.setPose(CustomAutoBuilder.getStartPose2d());
    return CustomAutoBuilder.getAutonCommand(drive);
    // return autoChooser.get();
    // return new L4Auton(m_Elevator, m_Arm);
  }
}
