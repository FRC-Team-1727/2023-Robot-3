// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDSim sim;
  private int lightMode;
  private double animStart;

  XboxController ledController = new XboxController(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    lightMode = 0;
    animStart = 0;
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();


    sim = new AddressableLEDSim(m_led);
    sim.setRunning(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    switch (lightMode) {
      case 0: //default
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        int value = ((int)animStart + (i * 255 / m_ledBuffer.getLength())) % 510;
        if (value > 255) {
          value = 510 - value;
        }
        m_ledBuffer.setHSV(i, DriverStation.getAlliance() == Alliance.Red ? 0 : 120, 255, value);
        
      }
      animStart += 7;
      animStart %= 510;
      break;
      case 1: //cone
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 255, 127, 0);
        }
        break;
      case 2: //cube
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setLED(i, Color.kPurple);
        }
        break;
      case 3: //party mode
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          int hue = ((int)animStart + (i * 180 / m_ledBuffer.getLength())) % 180;
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        animStart += 3;
        animStart %= 180;
        break;
    }

    m_led.setData(m_ledBuffer);

    if (ledController.getYButtonPressed()) {
      lightMode = 1;
    } else if (ledController.getBButtonPressed()) {
      lightMode = 2;
    } else if (ledController.getXButtonPressed()) {
      lightMode = 3;
      animStart = 0;
    } else if (ledController.getAButtonPressed()) {
      lightMode = 0;
      animStart = 0;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.driverInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
