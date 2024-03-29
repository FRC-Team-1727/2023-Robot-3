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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ElevatorSubsystem extends SubsystemBase {
  
  private CANSparkMax elevatorMotor = new CANSparkMax(kElevatorPort, MotorType.kBrushless);
  private CANSparkMax angler = new CANSparkMax(kAnglerPort, MotorType.kBrushless);

  private double elevation;
  private double angle;
  private int position;
  private int anglePosition;
  private boolean recovering;
  private boolean coneMode;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    angler.getPIDController().setFeedbackDevice(angler.getAbsoluteEncoder(Type.kDutyCycle));
    elevatorMotor.getPIDController().setFeedbackDevice(elevatorMotor.getEncoder());
    // angler.getPIDController().setFeedbackDevice(angler.getEncoder());
    elevation = 0;
    angle = 0;
    position = 0;
    anglePosition = 2;

    elevatorMotor.getPIDController().setP(0.1);
    elevatorMotor.getPIDController().setI(0);
    elevatorMotor.getPIDController().setD(0.1);
    elevatorMotor.getPIDController().setFF(0);
    angler.getPIDController().setP(kLowP); //2
    angler.getPIDController().setI(kLowI); //0.0001
    angler.getPIDController().setD(0.1); //0.525
    angler.getPIDController().setFF(0);
    angler.setInverted(true);
    // angler.getPIDController().setSmartMotionMaxAccel(5, 0);
    // angler.getPIDController().setSmartMotionMaxVelocity(5676, 0);

    angler.getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(4.0);

    elevatorMotor.getPIDController().setOutputRange(-1, 1);
    angler.getPIDController().setOutputRange(-0.70, 1);
    // angler.getPIDController().setOutputRange(-0.25, 0.25);
    // elevatorMotor.getPIDController().setOutputRange(-0.5, 0.5);


    elevatorMotor.setIdleMode(IdleMode.kCoast);
    angler.setIdleMode(IdleMode.kBrake);

    recovering = false;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

   public CommandBase driverInit() {
    return setAnglerNormalSpeed().andThen(drivePosition());
   }

   public CommandBase slowAngler() {
    return runOnce(
      ()-> {
        angler.getPIDController().setOutputRange(-0.1, 0.1);
      }
    );
   }

   public CommandBase setAnglerNormalSpeed() {
    return runOnce(
      ()-> {
        angler.getPIDController().setOutputRange(-1, 1);
      }
    );
   }


  public CommandBase updateElevator() {
    return runOnce(
      () -> {
        if (elevation < 0) {
          elevation = 0;
        }
        elevatorMotor.getPIDController().setReference(elevation - angle, ControlType.kPosition);

        if (position == 2) {
          angler.getPIDController().setP(kHighP);
          angler.getPIDController().setI(kHighI);
        } else {
          angler.getPIDController().setP(kLowP);
          angler.getPIDController().setI(kLowI);
        }

        if (recovering && angler.getAbsoluteEncoder(Type.kDutyCycle).getPosition() > angle) {
          moveAngle(0);
          setAnglePosition(anglePosition);
        }
      });
  }

  public CommandBase drivePosition() {
    return setPosition(()->0).andThen(setAnglePosition(()->2));
  }

  public void setDrivePosition() {
    setPosition(0);
    setAnglePosition(2);
  }

  public boolean atZero() {
    return elevatorMotor.getEncoder().getPosition() < 10;
  }

  public CommandBase intakePosition() {
    return setAnglePosition(()->0).andThen(setPosition(()->0))
    // .andThen(runOnce(
    //   ()-> elevatorMotor.getPIDController().setOutputRange(-0.45, 0.75)
    //   ))
      ;
  }

  public void setIntakePosition() {
    setPosition(0);
    setAnglePosition(0);
  }

  public CommandBase scoringPosition() {
    if (position == 2) return intakePosition();
    return changePosition().andThen(setAnglePosition(()->1));
  }

  public CommandBase loadingPosition() {
    return setAnglePosition(()->3).andThen(setPosition(()->3));
  }

  public void setLoadingPosition() {
    setAnglePosition(3);
    setPosition(3);
  }
  
  public void move(double value) {
    // elevation += value * kElevatorSpeed;
    elevatorMotor.set(value * kElevatorSpeed);
  }

  public CommandBase setPosition(IntSupplier position) {
    return runOnce(
      () -> {
        this.position = position.getAsInt();
        if (coneMode) {
          this.elevation = kElevatorPositions[position.getAsInt()];
        } else {
          this.elevation = kCubePositions[position.getAsInt()];
        }
        // System.out.println("setting position" + " " + elevation + " " + position);
      }).andThen(updateElevator());
  }

  public void setPosition(int position) {
    this.position = position;
    this.elevation = kElevatorPositions[position];
    if (elevation < 0) {
      elevation = 0;
    }
    elevatorMotor.getPIDController().setReference(elevation - angle, ControlType.kPosition);
  }

  public CommandBase changePosition() {
    return runOnce(
      () -> {
        position++;
        if (position >= kElevatorPositions.length - 1) {
          position = 1;
        }
        // elevatorMotor.getPIDController().setOutputRange(-0.75, 0.75);
      }).andThen(setPosition(() -> position));
  }

  public void moveAngle(double value) {
    // angle += value;
    // angler.getPIDController().setReference(-angle, ControlType.kPosition);
    angler.set(value * 0.05);
  }

  public CommandBase setAnglePosition(IntSupplier position) {
    return runOnce(
      () -> {
        setAnglePosition(position.getAsInt());
      });
  }

  public void setAnglePosition(int position) {
    if (!coneMode && position == 1) {
      angle = kCubeAngle;
    } else {
      angle = kAnglerPositions[position];
    }
    anglePosition = position;
    angler.getPIDController().setReference(angle, ControlType.kPosition);
    recovering = false;
  }

  public void runLowVoltage() {
    elevatorMotor.getPIDController().setReference(-1, ControlType.kVoltage);
  }

  public void zeroElevator() {
    elevatorMotor.stopMotor();
    elevatorMotor.getEncoder().setPosition(-3);
  }

  public boolean elevatorStopped() {
    return Math.abs(elevatorMotor.getEncoder().getVelocity()) < 0.1;
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

  public void setElevationAsIs() {
    elevation = elevatorMotor.getEncoder().getPosition();
  }

  public void setAngleAsIs() {
    angle = kAnglerPositions[anglePosition];
        angler.getPIDController().setReference(angle, ControlType.kPosition);
        if (position == 2) {
          angler.set(0.6);
          recovering = true;
        }
  }

  public CommandBase setAngle(DoubleSupplier angle) {
    return runOnce(
      () -> {
        this.angle = angle.getAsDouble();
        angler.getPIDController().setReference(this.angle, ControlType.kPosition);
      }
    );
  }

  public void setConeMode(boolean mode) {
    coneMode = mode;
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

  public boolean inDrivePosition() {
    return position == 0 && anglePosition == 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator encoder", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("angler encoder", angler.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    // SmartDashboard.putNumber("angler motor encoder", angler.getEncoder().getPosition());
    // SmartDashboard.putNumber("elevator target", elevation);
    // SmartDashboard.putNumber("angler target", angle);
    // SmartDashboard.putNumber("elevator voltage", elevatorMotor.getBusVoltage());
    // SmartDashboard.putNumber("elevator velocity", elevatorMotor.getEncoder().getVelocity());
    // SmartDashboard.putBoolean("elevator stopped", elevatorStopped());
    // SmartDashboard.putBoolean("elevator at zero", atZero());
    // SmartDashboard.putBoolean("recovering", recovering);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
