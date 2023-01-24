// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
  
  private CANSparkMax[] elevatorMotors = new CANSparkMax[] {
    new CANSparkMax(kElevatorPorts[0], MotorType.kBrushless),
    new CANSparkMax(kElevatorPorts[1], MotorType.kBrushless)
  };

  private CANSparkMax angler = new CANSparkMax(kAnglerPort, MotorType.kBrushless);

  private double position;
  private double angle;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotors[0].getPIDController().setFeedbackDevice(elevatorMotors[0].getAbsoluteEncoder(Type.kDutyCycle));
    elevatorMotors[1].follow(elevatorMotors[0], true);
    angler.getPIDController().setFeedbackDevice(angler.getAbsoluteEncoder(Type.kDutyCycle));
    position = 0;
    angle = 0;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase move(double value) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          position += value;
          elevatorMotors[0].getPIDController().setReference(position, ControlType.kPosition);
        });
  }

  public CommandBase movePosition(int position) {
    return runOnce(
      () -> {
        this.position = kElevatorPositions[position];
      });
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
