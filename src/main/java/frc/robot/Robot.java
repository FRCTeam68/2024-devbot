// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distMXP;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    System.out.println("robot init");

    /**
     * Rev 2m distance sensor can be initialized with the Onboard I2C port
     * or the MXP port. Both can run simultaneously.
     */
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kDefault);
    distMXP = new Rev2mDistanceSensor(Port.kMXP);

    m_robotContainer = new RobotContainer();


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
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

    // distOnboard.setMeasurementPeriod(0.020);
    /**
     * Before measurements can be read from the sensor, setAutomaticMode(true)
     * must be called. This starts a background thread which will periodically
     * poll all enabled sensors and store their measured range.
     */
    distOnboard.setAutomaticMode(true);

  }

  @Override
  public void teleopPeriodic() {

    if(distOnboard.isRangeValid()) {
      SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
      SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
    }

    if(distMXP.isRangeValid()) {
      SmartDashboard.putNumber("Range MXP", distMXP.getRange());
      SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
    }

    SmartDashboard.putNumber("number1", 10);

  }

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
