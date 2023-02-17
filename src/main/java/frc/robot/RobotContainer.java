// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getRightY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              false, true),
            m_robotDrive));

    m_intakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_intakeSubsystem.intake(
          ()->m_driverController.getRightTriggerAxis()),
          m_intakeSubsystem));

    // m_intakeSubsystem.setDefaultCommand(
    //     // intake with right trigger
    //     // outtake with left trigger
    //     new RunCommand(
    //         () -> m_intakeSubsystem.intake(
    //             m_driverController.getRightTriggerAxis(),
    //             m_driverController.getLeftTriggerAxis()),
    //         m_intakeSubsystem));

    //updates elevator position based on angle
    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.updateElevation());

    //manual control for testing
    // m_elevatorSubsystem.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_elevatorSubsystem.manualControl(
    //             m_driverController.leftTrigger().getAsBoolean(),
    //             m_driverController.leftBumper().getAsBoolean(),
    //             m_driverController.rightTrigger().getAsBoolean(),
    //             m_driverController.rightBumper().getAsBoolean()), m_elevatorSubsystem)
    // );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    m_driverController.rightTrigger().onTrue(m_elevatorSubsystem.intakePosition());
    m_driverController.rightBumper().onTrue(m_elevatorSubsystem.setAnglePosition(() -> 2));
    m_driverController.leftBumper().onTrue(m_elevatorSubsystem.changePosition());
    m_driverController.leftTrigger().whileTrue(new OuttakeCommand(
      m_elevatorSubsystem, m_intakeSubsystem,
      () -> -m_driverController.getLeftTriggerAxis(),
      () -> m_driverController.getRightY()
    ));
    m_driverController.a().onTrue(new ZeroElevatorCommand(m_elevatorSubsystem));

    /*tentative controls
     * RT - intake position down (angle horizontal, elevator out some) - also runs intake
     * RB - elevator up
     * LB - toggle elevator extension (close/far)
     * LT - outtake/retract elevator
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {      
    return Autos.highConeAuto(m_elevatorSubsystem);
    // return Autos.driveTestAuto(m_robotDrive);
  }
}
