// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  
  public static SwerveControllerCommand swerveCommand(DriveSubsystem m_robotDrive, ProfiledPIDController thetaController, Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
  }
  
  public static CommandBase highConeAuto(ElevatorSubsystem elevator, DriveSubsystem m_robotDrive, TrajectoryConfig config, ProfiledPIDController thetaController) {
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(Math.toRadians(179))),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(2, 0, new Rotation2d(0)),
      config);
    
    m_robotDrive.resetOdometry(t1.getInitialPose());
    m_robotDrive.resetGyro();
    return Commands.sequence(
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(2),
      elevator.setAngle(()->1.7),
      new WaitCommand(1),
      elevator.setPosition(()->0),
      new WaitCommand(1),
      elevator.intakePosition(),
      swerveCommand(m_robotDrive, thetaController, t1)
    );
  }

  public static CommandBase parkAuto(ElevatorSubsystem elevator, DriveSubsystem m_robotDrive, TrajectoryConfig config, ProfiledPIDController thetaController) {
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(Units.feetToMeters(8), 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(2, 0, new Rotation2d(180)),
      config);

      m_robotDrive.resetOdometry(t1.getInitialPose());
      m_robotDrive.resetGyro();
      return Commands.sequence(
        elevator.setAnglePosition(()->1),
        elevator.setPosition(()->2),
        new WaitCommand(2),
        elevator.setAngle(()->1.7),
        new WaitCommand(1),
        elevator.setPosition(()->0),
        new WaitCommand(1),
        elevator.intakePosition(),
        swerveCommand(m_robotDrive, thetaController, t1).alongWith(new ZeroElevatorCommand(elevator))
      );
  }

  public static CommandBase exampleAuto(DriveSubsystem m_robotDrive) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(Math.toRadians(179))),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    m_robotDrive.resetGyro();

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
