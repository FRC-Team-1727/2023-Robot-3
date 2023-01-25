// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
  
  private CANSparkMax elevatorMotor = new CANSparkMax(kElevatorPort, MotorType.kBrushless);
  private CANSparkMax angler = new CANSparkMax(kAnglerPort, MotorType.kBrushless);

  private double elevation;
  private double angle;
  private int position;
  private int anglePosition;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor.getPIDController().setFeedbackDevice(elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle));
    angler.getPIDController().setFeedbackDevice(angler.getAbsoluteEncoder(Type.kDutyCycle));
    elevation = 0;
    angle = 0;
    position = 0;
    anglePosition = 2;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  private void moveToElevation() {
    elevatorMotor.getPIDController().setReference(elevation - angle, ControlType.kPosition);
  }
  
   public CommandBase move(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          elevation += value * kElevatorSpeed;
          moveToElevation();
        });
  }

  public CommandBase setPosition(int position) {
    return runOnce(
      () -> {
        this.elevation = kElevatorPositions[position];
        moveToElevation();
      });
  }

  public CommandBase changePosition() {
    position++;
    if (position >= kElevatorPositions.length) {
      position = 1;
    }

    return setPosition(position);
  }

  public CommandBase moveAngle(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          angle += value;
          angler.getPIDController().setReference(angle, ControlType.kPosition);
        });
  }

  public CommandBase setAnglePosition(int position) {
    return runOnce(
      () -> {
        angle = kAnglerPositions[position];
        anglePosition = position;
        angler.getPIDController().setReference(angle, ControlType.kPosition);
      });
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
