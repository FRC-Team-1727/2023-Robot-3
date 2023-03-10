// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intake;
  private final ElevatorSubsystem elevator;
  private final boolean loading;
  private final DoubleSupplier speed;

  /**
   * Creates a new IntakeCommand.
   *
   * @param intake The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem intake, ElevatorSubsystem elevator, DoubleSupplier speed) {
    this.intake = intake;
    this.elevator = elevator;
    this.speed = speed;
    loading = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, elevator);
  }

  public IntakeCommand(IntakeSubsystem intake, ElevatorSubsystem elevator, boolean loading) {
    this.intake = intake;
    this.elevator = elevator;
    this.loading = loading;
    speed = ()->1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setDrivePosition();
    SmartDashboard.putString("intake command", "starting");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.atZero()) {
      if (loading) {
        elevator.setLoadingPosition();
      } else {
        elevator.setIntakePosition();
      }
    }
    intake.intake(speed, ()->true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setDrivePosition();
    // intake.intake(()->0);
    SmartDashboard.putString("intake command", "ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
