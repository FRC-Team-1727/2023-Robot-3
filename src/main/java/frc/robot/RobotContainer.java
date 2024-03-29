// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
              true, true),
            m_robotDrive));

    m_intakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_intakeSubsystem.intake(
          ()->m_driverController.getRightTriggerAxis() + (m_driverController.rightBumper().getAsBoolean() ? 2 : 0),
          ()->m_driverController.povCenter().getAsBoolean(),
          ()->m_elevatorSubsystem.inDrivePosition()),
          m_intakeSubsystem));

    //updates elevator position & angle
    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.updateElevator());

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
    
    // m_driverController.rightTrigger().onTrue(m_elevatorSubsystem.intakePosition());
    // m_driverController.rightBumper().onTrue(m_elevatorSubsystem.drivePosition());
    // m_driverController.y().onTrue(m_elevatorSubsystem.loadingPosition());
    m_driverController.leftBumper().onTrue(m_elevatorSubsystem.scoringPosition());
    m_driverController.leftTrigger().whileTrue(new ScoreCommand(
      m_elevatorSubsystem, m_intakeSubsystem,
      () -> -m_driverController.getLeftTriggerAxis(),
      () -> m_driverController.getRightY()
    ));
    m_driverController.a().onTrue(new ZeroElevatorCommand(m_elevatorSubsystem));
    m_driverController.b().onTrue(m_robotDrive.runOnce(()->m_robotDrive.resetGyro(0)));
    // m_driverController.back().onTrue(m_robotDrive.startSnapping());
    // m_driverController.rightBumper().onTrue(m_robotDrive.startSnapping(()->m_driverController.getRightX(), ()->m_driverController.getRightY()));

    m_driverController.rightTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem, m_elevatorSubsystem, ()->m_driverController.getRightTriggerAxis()));
    // m_driverController.rightTrigger().onFalse(m_elevatorSubsystem.drivePosition());
    m_driverController.rightBumper().whileTrue(new IntakeCommand(m_intakeSubsystem, m_elevatorSubsystem, true));
    // m_driverController.x().onTrue(new RunCommand(()->m_robotDrive.resetGyro(0), m_robotDrive));
    /* controls:
     * RS - forward, backward, strafe
     * LS - turning
     * RT - run intake, move elevator into drive position until elevator at 0, then intake position. Move to drive position when released.
     * LT - slowly lower angle, outtake while backing up?
     * RB - same as RT, except match loading position rather than intake position.
     * LB - toggle between scoring positions
     */

  }

  public void driverInit() {
    m_elevatorSubsystem.driverInit().schedule();
    setConeMode(true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int auto) {
    Autos.loadEventMap(m_elevatorSubsystem, m_robotDrive, m_intakeSubsystem);

    switch (auto) {
      case 0: return Autos.redLoadingPark(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 1: return Autos.redLoadingTwoHalf(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 2: return Autos.blueLoadingPark(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 3: return Autos.blueLoadingTwoHalf(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 4: return Autos.middlePark(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 5: return Commands.none();
      case 6: return Autos.cableTwoHalf(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 7: return Autos.blueLoadingThree(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 8: return Autos.highConeFast(m_elevatorSubsystem, m_intakeSubsystem);
      case 9: return Autos.middleTwo(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 10: return Autos.throwFar(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 11: return new BalanceCommand(m_robotDrive);
      case 12: return Autos.cableTwoPark(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
      case 13: return Autos.threeCone(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);

      default:
        return Autos.middlePark(m_elevatorSubsystem, m_intakeSubsystem, m_robotDrive);
    }
    
  }

  public boolean holdingObject() {
    return m_intakeSubsystem.holdingObject();
  }

  public void setConeMode(boolean mode) {
    m_intakeSubsystem.setConeMode(mode);
    m_elevatorSubsystem.setConeMode(mode);
  }

  public void setDoubleLoading(boolean shooting) {
    m_intakeSubsystem.setDoubleLoading(shooting);
  }
}
