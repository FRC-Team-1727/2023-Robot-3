// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase highConeAuto(ElevatorSubsystem elevator) {
    return Commands.sequence(elevator.setAnglePosition(()->2), elevator.setPosition(()->3), new WaitCommand(2),
    elevator.setAngle(()->0.3), new WaitCommand(1), elevator.intakePosition(), new WaitCommand(1),
    elevator.setAngle(()->0));
  }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
