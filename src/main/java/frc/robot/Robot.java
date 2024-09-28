package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.subsystems.Vision.VisionType;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private Dashboard dashboard;

  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    robotContainer = new RobotContainer();
    dashboard = new Dashboard();
    dashboard.loadDashboard(robotContainer.getSubsystems());
    robotContainer.getSubsystems().getVision().setPipeline(VisionType.TAG, 0);
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.getSubsystems().updatePosition();
    dashboard.updateDashboard(robotContainer.getSubsystems());
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
    robotContainer.getSubsystems().getVision().setPipeline(VisionType.TAG, 0);
    autonomousCommand = new WaitCommand(0.010).andThen(dashboard.getAutonomousCommand());

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // robotContainer.getSubsystems().getMusic().playSong("jackSparrow");
  }

  @Override
  public void teleopPeriodic() {
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
