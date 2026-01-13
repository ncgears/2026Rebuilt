// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Creates the robot and initializes the container. */
  public Robot() {
    m_robotContainer = new RobotContainer();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Runs the scheduler and periodic tasks each loop. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  /** Runs once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.neutralRobot();
  }

  /** Runs periodically while the robot is disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Runs once when exiting disabled mode. */
  @Override
  public void disabledExit() {}

  /** Schedules the selected autonomous command. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** Runs periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** Runs once when exiting autonomous. */
  @Override
  public void autonomousExit() {}

  /** Cancels autonomous when teleop starts. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** Runs periodically during teleop. */
  @Override
  public void teleopPeriodic() {}

  /** Runs once when exiting teleop. */
  @Override
  public void teleopExit() {}

  /** Cancels all commands when test starts. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Runs periodically during test. */
  @Override
  public void testPeriodic() {}

  /** Runs once when exiting test. */
  @Override
  public void testExit() {}

  /** Runs periodically during simulation. */
  @Override
  public void simulationPeriodic() {}
}
