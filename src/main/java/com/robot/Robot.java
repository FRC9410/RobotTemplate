// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    m_robotContainer = new RobotContainer();
    autoChooser.setDefaultOption("Default Auto", "Default");
    autoChooser.addOption("Auto Mode 1", "Auto1");
    autoChooser.addOption("Auto Mode 2", "Auto2");
    autoChooser.addOption("Auto Mode 3", "Auto3");

    SmartDashboard.putData("Auto choices", autoChooser);
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
    String selectedAuto = autoChooser.getSelected();
    System.out.println("Selected Auto: " + selectedAuto);

    switch (selectedAuto) {
        case "Auto1":
            break;
        case "Auto2":
            break;
        case "Auto3":
            break;
        default:
            break;
    }

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
}
