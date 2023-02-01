// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  
  private CANSparkMax elevatorMotor = new CANSparkMax(kElevatorPort, MotorType.kBrushless);
  private CANSparkMax angler = new CANSparkMax(kAnglerPort, MotorType.kBrushless);

  private double elevation;
  private double angle;
  private int position;
  private int anglePosition;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // elevatorMotor.getPIDController().setFeedbackDevice(elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle));
    // angler.getPIDController().setFeedbackDevice(angler.getAbsoluteEncoder(Type.kDutyCycle));
    elevatorMotor.getPIDController().setFeedbackDevice(elevatorMotor.getEncoder());
    angler.getPIDController().setFeedbackDevice(angler.getEncoder());
    elevation = 0;
    angle = 0;
    position = 0;
    anglePosition = 2;

    elevatorMotor.getPIDController().setP(1);
    elevatorMotor.getPIDController().setI(0);
    elevatorMotor.getPIDController().setD(0);
    angler.getPIDController().setP(1);
    angler.getPIDController().setI(0);
    angler.getPIDController().setD(0);

    elevatorMotor.getPIDController().setOutputRange(-0.25, 0.25);
    angler.getPIDController().setOutputRange(-0.25, 0.25);


    elevatorMotor.setIdleMode(IdleMode.kCoast);
    angler.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public CommandBase updateElevation() {
    return runOnce(
      () -> {
        if (elevation < 0) {
          elevation = 0;
        }
        elevatorMotor.getPIDController().setReference(elevation - angle, ControlType.kPosition);
      });
  }
  
  public void move(double value) {
    elevation += value * kElevatorSpeed;
  }

  public CommandBase setPosition(IntSupplier position) {
    return runOnce(
      () -> {
        this.elevation = kElevatorPositions[position.getAsInt()];
        System.out.println("setting position" + " " + elevation + " " + position);
      });
  }

  public CommandBase changePosition() {
    return runOnce(
      () -> {
        position++;
        if (position >= kElevatorPositions.length) {
          position = 1;
        }
      }).andThen(setPosition(() -> position));
  }

  public void moveAngle(double value) {
    angle += value;
    angler.getPIDController().setReference(-angle, ControlType.kPosition);
  }

  public CommandBase setAnglePosition(IntSupplier position) {
    return runOnce(
      () -> {
        angle = kAnglerPositions[position.getAsInt()];
        anglePosition = position.getAsInt();
        angler.getPIDController().setReference(-angle, ControlType.kPosition);
        // System.out.println("setting angle");
      });
  }

  public CommandBase zeroEncoders() {
    return runOnce(
      () -> {
        elevatorMotor.getEncoder().setPosition(0);
        angler.getEncoder().setPosition(0);
      }
    );
  }

  public CommandBase intakePosition() {
    return setAnglePosition(()->1).andThen(setPosition(()->1));
  }

  public void manualControl(boolean lt, boolean lb, boolean rt, boolean rb) {
    if(lt) {
      elevatorMotor.set(-0.6);
    } else if (lb) {
      elevatorMotor.set(0.6);
    } else {
      elevatorMotor.stopMotor();
    }

    if (rt) {
      angler.set(0.1);
    } else if (rb) {
      angler.set(-0.1);
    } else {
      angler.stopMotor();
    }
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
    SmartDashboard.putNumber("elevator encoder", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("angler encoder", angler.getEncoder().getPosition());
    SmartDashboard.putNumber("elevator target", elevation);
    SmartDashboard.putNumber("angler target", angle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
