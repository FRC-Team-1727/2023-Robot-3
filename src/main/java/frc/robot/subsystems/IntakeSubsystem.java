// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax intake = new CANSparkMax(kIntakePort, MotorType.kBrushless);
  private SparkMaxPIDController intakeController = intake.getPIDController();
  private double currentPosition;
  
  public IntakeSubsystem() {
    intake.setIdleMode(IdleMode.kBrake);
    intakeController.setP(kP);
    intakeController.setP(kI);
    intakeController.setP(kD);
    intakeController.setP(kFF);
    intakeController.setFeedbackDevice(intake.getEncoder());
    intakeController.setOutputRange(-1, 1);
    // intake.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  
  public void intake(DoubleSupplier spd, BooleanSupplier outtaking) {
    if (!outtaking.getAsBoolean()) {
      // intakeController.setReference(-kOuttakeSpeed, ControlType.kVelocity);
      // System.out.println("outtaking");
      outtake(1);
    } else 
    if (spd.getAsDouble() < 0.1) {
      // intakeController.setReference(currentPosition, ControlType.kPosition);
      intake.set(0.05);
      // System.out.println("passive");
    } else {
      intake.set(spd.getAsDouble() * kIntakeSpeed);
      currentPosition = intake.getEncoder().getPosition();
      // intakeController.setReference(kIntakeSpeed, ControlType.kVelocity);
      // System.out.println("intaking");
    }

    // System.out.println(outtaking.getAsBoolean());
  }

  public void outtake(double spd) {
      intake.set(spd * kOuttakeSpeed);
  }

  public void stop() {
    intake.stopMotor();
  }

  public CommandBase intakeCommand(DoubleSupplier spd) {
    return runOnce(()->intake(spd, ()->true));
  }

  public CommandBase outtakeCommand(DoubleSupplier spd) {
    return runOnce(()->outtake(spd.getAsDouble()));
  }
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake target", currentPosition);
    SmartDashboard.putNumber("intake position", intake.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
