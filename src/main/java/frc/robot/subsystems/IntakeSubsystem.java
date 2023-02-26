// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intake = new CANSparkMax(kIntakePort, MotorType.kBrushless);
  
  public IntakeSubsystem() {
    intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  
  
  public void intake(DoubleSupplier spd) {
    if (spd.getAsDouble() < 0.1) {
      stop();
    } else {
      intake.set(spd.getAsDouble() * kIntakeSpeed);
    }
  }

  public void outtake(double spd) {
      intake.set(spd * kOuttakeSpeed);
  }

  public void stop() {
    intake.stopMotor();
  }

  public CommandBase intakeCommand(DoubleSupplier spd) {
    return runOnce(()->intake(spd));
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
