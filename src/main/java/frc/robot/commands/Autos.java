// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  
  public static CommandBase highConeAuto(ElevatorSubsystem elevator, DriveSubsystem m_robotDrive, IntakeSubsystem intake, TrajectoryConfig config, ProfiledPIDController thetaController) {
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(2, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(Units.feetToMeters(16), 0, new Rotation2d(0)),
      config);

    Trajectory t2 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(Units.feetToMeters(16), 0, Rotation2d.fromDegrees(180)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
        new Translation2d(Units.inchesToMeters(4), 0),
        new Translation2d(Units.inchesToMeters(4), Units.feetToMeters(4))
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, Units.feetToMeters(4), new Rotation2d(0)),
      config);
    
    m_robotDrive.resetOdometry(t1.getInitialPose());
    m_robotDrive.resetGyro(t1.getInitialPose().getRotation().getDegrees());
    return Commands.sequence(
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(2),
      elevator.setAngle(()->1.8),
      new WaitCommand(1),
      elevator.setPosition(()->0),
      new WaitCommand(1),
      elevator.intakePosition(),
      intake.intakeCommand(()->1),
      swerveCommand(m_robotDrive, thetaController, t1).alongWith(new ZeroElevatorCommand(elevator)),
      intake.intakeCommand(()->0),
      elevator.drivePosition(),
      swerveCommand(m_robotDrive, thetaController, t2),
      elevator.setAnglePosition(()->1),
      elevator.setPosition(()->2),
      new WaitCommand(2),
      elevator.setAngle(()->1.8),
      new WaitCommand(1),
      elevator.setPosition(()->0),
      new WaitCommand(1),
      elevator.intakePosition()

    );
  }

  public static CommandBase parkAuto(ElevatorSubsystem elevator, DriveSubsystem m_robotDrive, IntakeSubsystem intake, TrajectoryConfig config, ProfiledPIDController thetaController) {
    Trajectory t1 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(Units.feetToMeters(8), 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(Units.feetToMeters(14), 0, Rotation2d.fromDegrees(179)),
      config);

    Trajectory t2 = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(Units.feetToMeters(14), 0, Rotation2d.fromDegrees(179)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(Units.feetToMeters(10), 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(Units.feetToMeters(10), 0, Rotation2d.fromDegrees(178)),
      config);

      m_robotDrive.resetOdometry(t1.getInitialPose());
      m_robotDrive.resetGyro(180);
      return Commands.sequence(
        m_robotDrive.runOnce(() -> m_robotDrive.drive(0, 0, 0, false, false)),
        elevator.setAnglePosition(()->1),
        elevator.setPosition(()->2),
        new WaitCommand(2),
        elevator.setAngle(()->1.8),
        new WaitCommand(1),
        intake.intakeCommand(()->-0.2),
        elevator.setPosition(()->1),
        new WaitCommand(1),
        intake.intakeCommand(()->0),
        elevator.drivePosition(),
        // swerveCommand(m_robotDrive, thetaController, t1),
        // swerveCommand(m_robotDrive, thetaController, t2),
        m_robotDrive.runOnce(() -> m_robotDrive.drive(0, 0, 0, false, false))
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
    m_robotDrive.resetGyro(0);

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
