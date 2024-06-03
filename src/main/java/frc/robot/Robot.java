package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.subsystems.Subsystems.PeriodType;
import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.Utility;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private Dashboard dashboard;
  private Command lastWarmedUpCommand;

  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    robotContainer = new RobotContainer();
    dashboard = new Dashboard();
    dashboard.loadDashboard(robotContainer.getSubsystems());
    robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 0);
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    dashboard.updateDashboard(robotContainer.getSubsystems());

    boolean hasTarget = robotContainer.getSubsystems().getVision().hasTarget(VisionType.TARGET);

    if(hasTarget) {
      robotContainer.getSubsystems().getLeds().setFadeAnimtation(255, 121, 198);
    }
    else {
      robotContainer.getSubsystems().getLeds().setFadeAnimtation(0, 255, 255);
    }
  } 

  @Override
  public void disabledInit() {
    robotContainer.getSubsystems().setDisabledIdleMode();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    robotContainer.getSubsystems().setEnabledIdleMode();
  }

  @Override
  public void autonomousInit() {
    robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 0);
    autonomousCommand = new WaitCommand(0.010).andThen(dashboard.getAutonomousCommand());

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.getSubsystems().updatePosition(PeriodType.AUTO);
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    var alliance = Utility.getAllianceColor();
    if (alliance == "red") {
      robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 1);
    }
    else {
      robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 2);
    }
    // robotContainer.getSubsystems().getMusic().playSong("jackSparrow");
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.getSubsystems().updatePosition(PeriodType.TELEOP);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {}
}
