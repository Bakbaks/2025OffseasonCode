// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogManager;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer();

  }

  private static final double LOG_PERIOD = 0.02; // 50Hz logging
  private double lastLogTime = 0;
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry memoryEntry = new DoubleLogEntry(log, "/memory/free");

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastLogTime >= LOG_PERIOD) {
      // Vision fusion with CTRE Phoenix Kalman filter
      m_robotContainer.fuseVisionMeasurements();
      
      // Log memory usage
      Runtime runtime = Runtime.getRuntime();
      double freeMemoryMB = runtime.freeMemory() / (1024.0 * 1024.0);
      double totalMemoryMB = runtime.totalMemory() / (1024.0 * 1024.0);
      memoryEntry.append(freeMemoryMB);
      SmartDashboard.putNumber("Free Memory (MB)", freeMemoryMB);
      SmartDashboard.putNumber("Total Memory (MB)", totalMemoryMB);
      
      lastLogTime = currentTime;
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}


  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}