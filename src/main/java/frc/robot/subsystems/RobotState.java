// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

public class RobotState extends SubsystemBase {

  private final CommandSwerveDrivetrain driveTrain;
  private final Vision vision;
  private CommandXboxController controller;

  private State state;
  private double locationX;
  private double locationY;
  private double rotation;

  private double targetX;
  private double targetY;
  private Optional<Rotation2d> targetRotation;

  private boolean isFollowingPath;

  public RobotState(CommandSwerveDrivetrain driveTrain, Vision vision, CommandXboxController controller) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    this.controller = controller;

    state = State.IDLE;
    locationX = -1;
    locationY = -1;
    rotation = 0;
    targetX = -1;
    targetY = -1;
    targetRotation = null;
    isFollowingPath = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d pose = driveTrain.getPose();
    locationX = pose.getX();
    locationY = pose.getY();
    rotation = pose.getRotation().getDegrees();
    isFollowingPath = controller.a().getAsBoolean();
  }

  public State getState() {
    return state;
  }

  public double getLocationX() {
    return locationX;
  }

  public double getLocationY() {
    return locationY;
  }

  public double getRotation() {
    return rotation;
  }

  public double getTargetX() {
    return targetX;
  }

  public double getTargetY() {
    return targetY;
  }

  public Optional<Rotation2d> getTargetRotation() {
    return targetRotation;
  }

  public boolean getIsFollowingPath() {
    return isFollowingPath;
  }

  public enum State {
      IDLE
  }
}
