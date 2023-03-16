// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ScoreCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final DoubleSupplier speed;
  private final DoubleSupplier rs;

  /**
   * Creates a new OuttakeCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public ScoreCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, DoubleSupplier lt, DoubleSupplier rs) {
    m_elevatorSubsystem = elevator;
    m_intakeSubsystem = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    speed = lt;
    this.rs = rs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.move(speed.getAsDouble());
    if (m_elevatorSubsystem.atZero()) {
      m_elevatorSubsystem.setIntakePosition();
    } else {
        m_elevatorSubsystem.moveAngle(speed.getAsDouble());
    }

    // if (rs.getAsDouble() > 0.1) {
    //   m_intakeSubsystem.outtake(rs.getAsDouble());
    // } else {
    //   m_intakeSubsystem.stop();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_elevatorSubsystem.atZero()) {
      m_elevatorSubsystem.setDrivePosition();
    } else {
      m_elevatorSubsystem.move(0);
      m_elevatorSubsystem.setElevationAsIs();
      m_elevatorSubsystem.moveAngle(0);
      m_elevatorSubsystem.setAngleAsIs();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
