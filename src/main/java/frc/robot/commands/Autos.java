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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

 

public final class Autos {
  private static HashMap<String, Command> eventMap = new HashMap<>();


  public static void loadEventMap(ElevatorSubsystem elevator, DriveSubsystem drive, IntakeSubsystem intake) {
    eventMap = new HashMap<>();
    eventMap.put("firstCone", highConeAuto(elevator, intake));
    eventMap.put("intake", intake.intakeCommand(()->1));
    eventMap.put("stopIntake", intake.intakeCommand(()->0));
    eventMap.put("drivePosition", elevator.drivePosition());
    eventMap.put("intakePosition", elevator.intakePosition());
    eventMap.put("angleScore", elevator.setAnglePosition(()->1));
    eventMap.put("elevator0", elevator.setPosition(()->0));
    eventMap.put("elevator1", elevator.setPosition(()->1));
    eventMap.put("elevator2", elevator.setPosition(()->2));
    eventMap.put("scoreHigh", scoreHigh(elevator, intake));
    eventMap.put("firstConeSafe", highConeSafe(elevator, intake));
    eventMap.put("setX", new RunCommand(()->drive.setX(), drive));
  }

  public static CommandBase highConeAuto(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    CommandBase auto = Commands.sequence(
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(1.5),
      scoreHigh(elevator, intake)
    );

    auto.addRequirements(intake);
    return auto;
  }

  public static CommandBase highConeSafe(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    CommandBase auto = Commands.sequence(
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(1.5),
      scoreHighSafe(elevator, intake)
    );

    auto.addRequirements(intake);
    return auto;
  }

  public static CommandBase scoreHigh(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    return Commands.sequence(
      elevator.slowAngler(),
      elevator.setAngle(()->2.0),
      new WaitCommand(0.5),
      elevator.setPosition(()->0),
      intake.outtakeCommand(()->2),
      new WaitCommand(0.3),
      elevator.drivePosition(),
      intake.intakeCommand(()->0),
      elevator.setAnglerNormalSpeed()
    );
  }

  public static CommandBase scoreHighSafe(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    return Commands.sequence(
      elevator.slowAngler(),
      elevator.setAngle(()->2.0),
      new WaitCommand(1),
      elevator.setPosition(()->0),
      intake.outtakeCommand(()->1),
      new WaitCommand(0.5),
      elevator.drivePosition(),
      intake.intakeCommand(()->0),
      elevator.setAnglerNormalSpeed()
    );
  }

  public static CommandBase middlePark(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("park", new PathConstraints(1, 1), new PathConstraints(0.5, 1));

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public static CommandBase redLoadingThree(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("red_loading_three", new PathConstraints(3, 3), new PathConstraints(2, 2));

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public static CommandBase blueLoadingThree(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("two_half_piece_blue", new PathConstraints(3, 3), new PathConstraints(2, 2));

    drive.resetGyro(0);

    CustomAutoBuilder autoBuilder = new CustomAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }
  
  public static CommandBase redLoadingPark(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("red_loading_park", new PathConstraints(3, 3), new PathConstraints(2, 2), new PathConstraints(1, 1));

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public static CommandBase blueLoadingPark(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("blue_loading_park", new PathConstraints(3, 3), new PathConstraints(2, 2), new PathConstraints(1, 1));

    drive.resetGyro(0);

    CustomAutoBuilder autoBuilder = new CustomAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
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
