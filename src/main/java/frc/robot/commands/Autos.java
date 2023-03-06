// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

 

public final class Autos {
  private static HashMap<String, Command> eventMap = new HashMap<>();


  public static void loadEventMap(ElevatorSubsystem elevator, DriveSubsystem drive, IntakeSubsystem intake) {
    eventMap = new HashMap<>();
    eventMap.put("firstCone", highConeAuto(elevator, intake));
    eventMap.put("intake", intake.intakeCommand(()->1));
    eventMap.put("stopIntake", intake.intakeCommand(()->0));
    eventMap.put("drivePosition", elevator.drivePosition());
    eventMap.put("angleScore", elevator.setAnglePosition(()->1));
    eventMap.put("elevator0", elevator.setPosition(()->0));
    eventMap.put("elevator1", elevator.setPosition(()->1));
    eventMap.put("elevator2", elevator.setPosition(()->2));
  }

  public static CommandBase highConeAuto(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    CommandBase auto = Commands.sequence(
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(2),
      elevator.setAngle(()->1.8),
      new WaitCommand(1),
      elevator.setPosition(()->0),
      intake.outtakeCommand(()->-1),
      new WaitCommand(1),
      elevator.intakePosition(),
      intake.intakeCommand(()->0)
    );

    auto.addRequirements(intake);
    return auto;
  }

  public static CommandBase climbAuto(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Climb", new PathConstraints(1, 1));

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public static CommandBase testAuto(DriveSubsystem drive) {
    PathPlannerTrajectory pathGroup = PathPlanner.loadPath("test", new PathConstraints(1, 1));
    HashMap<String, Command> eventMap = new HashMap<>();

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public static CommandBase threePieceAuto(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("three_piece", new PathConstraints(1, 1));

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
