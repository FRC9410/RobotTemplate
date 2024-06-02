package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Utility;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    robotContainer = new RobotContainer();
    
    var alliance = Utility.getAllianceColor();
    if (alliance == "red") {
      robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 1);
    }
    else {
      robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 2);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    // robotContainer.setDisabledIdleMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    // robotContainer.setEnabledIdleMode();
  }

  @Override
  public void autonomousInit() {
    robotContainer.getSubsystems().getVision().setPipeline(VisionType.TARGET, 0);
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-back");
    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue("limelight-back");
    if (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagArea > 0.2) {
      Pose2d newPose = pose.toPose2d();
      newPose.rotateBy(Rotation2d.fromDegrees(180));
      robotContainer.getSubsystems().getDrivetrain().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      robotContainer.getSubsystems().getDrivetrain().addVisionMeasurement(
        newPose,
        limelightMeasurement.timestampSeconds
      );
      robotContainer.getSubsystems().getDrivetrain().seedFieldRelative(newPose);
    } 
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
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-back");
    Pose3d pose = Utility.getAllianceColor() == "red"
      ? LimelightHelpers.getBotPose3d_wpiRed("limelight-back")
      : LimelightHelpers.getBotPose3d_wpiBlue("limelight-back");
      
    if (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagArea > 0.1) {
      Pose2d newPose = pose.toPose2d();
      newPose.rotateBy(Rotation2d.fromDegrees(180));
      robotContainer.getSubsystems().getDrivetrain().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      robotContainer.getSubsystems().getDrivetrain().addVisionMeasurement(
        newPose,
        limelightMeasurement.timestampSeconds
      );
      robotContainer.getSubsystems().getDrivetrain().seedFieldRelative(newPose);
    }
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
