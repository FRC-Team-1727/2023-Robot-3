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
    eventMap.put("outtake", intake.outtakeCommand(()->1));
    eventMap.put("drivePosition", elevator.drivePosition());
    eventMap.put("intakePosition", elevator.intakePosition());
    eventMap.put("angleScore", elevator.setAnglePosition(()->1));
    eventMap.put("elevator0", elevator.setPosition(()->0));
    eventMap.put("elevator1", elevator.setPosition(()->1));
    eventMap.put("elevator2", elevator.setPosition(()->2));
    eventMap.put("scoreHigh", scoreHigh(elevator, intake));
    eventMap.put("firstConeFast", highConeFast(elevator, intake));
    eventMap.put("setX", new RunCommand(()->drive.setX(), drive));
    eventMap.put("throw", throwCube(elevator, intake));
    eventMap.put("throwFar", throwFar(elevator, intake, drive));
    eventMap.put("balance", new BalanceCommand(drive));
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

  public static CommandBase highConeFast(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    CommandBase auto = Commands.sequence(
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(0.75),
      scoreHighFast(elevator, intake)
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

  public static CommandBase scoreHighFast(ElevatorSubsystem elevator, IntakeSubsystem intake) {
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

  public static CommandBase throwCube(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    return Commands.sequence(
      intake.setSpeed(()->0.5),
      new WaitCommand(0.25),
      intake.setSpeed(()->-1),
      new WaitCommand(1),
      intake.intakeCommand(()->0)
    );
  }

  public static CommandBase throwFar(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    CommandBase result = Commands.sequence(
      drive.setXCommand(),
      intake.setSpeed(()->0.5),
      elevator.setPosition(()->1),
      new WaitCommand(0.35),
      intake.setSpeed(()->-1),
      new WaitCommand(0.5),
      intake.intakeCommand(()->0),
      elevator.drivePosition()
    );
    result.addRequirements(drive);
    return result;
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

  public static CommandBase redLoadingTwoHalf(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("red_loading_two_half", new PathConstraints(3, 3), new PathConstraints(2, 2));

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

  public static CommandBase blueLoadingTwoHalf(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("blue_loading_two_half", new PathConstraints(3, 3), new PathConstraints(2, 2));

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

  public static CommandBase cableTwoHalf(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("cable_two_half", new PathConstraints(3, 2.5), new PathConstraints(1, 1), new PathConstraints(2, 1.5), new PathConstraints(1, 1), new PathConstraints(2, 2));

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

  public static CommandBase blueLoadingThree(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("blue_loading_three", new PathConstraints(4, 4));

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

  public static CommandBase blueLoadingTwoPark(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("blue_loading_two_park", new PathConstraints(4, 4), new PathConstraints(1, 1));

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

  public static CommandBase middleTwo(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("middle_two", new PathConstraints(1.25, 2), new PathConstraints(3, 3), new PathConstraints(2, 2), new PathConstraints(1, 1));

    drive.resetGyro(0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getPose, // Pose2d supplier
      drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      DriveConstants.kDriveKinematics, // SwerveDriveKinematics
      new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(1.25, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drive::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);
  }

  public static CommandBase cableTwoPark(ElevatorSubsystem elevator, IntakeSubsystem intake, DriveSubsystem drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("cable_two_park", new PathConstraints(3, 2.5), new PathConstraints(2, 2), new PathConstraints(3, 2.5), new PathConstraints(2, 2), new PathConstraints(2, 2));

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

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
