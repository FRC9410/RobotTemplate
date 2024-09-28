package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.RobotState;

import frc.team9410.lib.Utility;

import java.util.List;

public class DefaultDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  RobotState robotState;
  Command followPathCommand;

  public DefaultDriveCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, RobotState robotState) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.robotState = robotState;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    if (robotState.getIsFollowingPath()) {
      followTrajectory();
    }
    else if (robotState.getTargetRotation().isPresent()){
      followPathCommand.cancel();
      drivetrain.drive(
        Utility.getSpeed(controller.getLeftY()) * DriveConstants.MaxSpeed,
        Utility.getSpeed(controller.getLeftX()) * DriveConstants.MaxSpeed,
        robotState.getTargetRotation(),
        DriveMode.FIELD_RELATIVE);
    }
    else {
      followPathCommand.cancel();
      drivetrain.drive(
        Utility.getSpeed(controller.getLeftY()) * DriveConstants.MaxSpeed,
        Utility.getSpeed(controller.getLeftX()) * DriveConstants.MaxSpeed,
        Utility.getSpeed(controller.getRightX()) * DriveConstants.MaxSpeed,
        DriveMode.FIELD_RELATIVE);
    }
  } 

  private void followTrajectory() {
    Rotation2d targetRotation = robotState.getTargetRotation().isPresent()
    ? robotState.getTargetRotation().get()
    : Rotation2d.fromDegrees(robotState.getRotation());
    
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(robotState.getTargetX(), robotState.getTargetY(), Rotation2d.fromDegrees(90))
    );

    PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
          new GoalEndState(0.0, targetRotation)
    );
    
    followPathCommand = AutoBuilder.followPath(path);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }
}