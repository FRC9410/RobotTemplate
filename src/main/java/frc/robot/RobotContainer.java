package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.Subsystems;
import frc.robot.commands.base.DefaultDriveCommand;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);
  private Subsystems subsystems = new Subsystems(driverController);
  // private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);
  private boolean devMode = false;

  public RobotContainer() {
    // subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);
    configurePilotBindings();
    configureCopilotBindings();
  }

  private void configurePilotBindings() {
    driverController.start().onTrue(subsystems.getDrivetrain().runOnce(() -> {
      subsystems.getDrivetrain().resetPose();
      subsystems.getDrivetrain().seedFieldRelative();
    }));
    
    subsystems.getDrivetrain().setDefaultCommand(
      new DefaultDriveCommand(
        subsystems.getDrivetrain(),
        driverController,
        subsystems.getRobotState()));
  }
  
  private void configureCopilotBindings() {
    
    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    // copilotController.back().and(copilotController.y()).whileTrue(subsystems.getDrivetrain().sysIdDynamic(Direction.kForward));
    // copilotController.back().and(copilotController.x()).whileTrue(subsystems.getDrivetrain().sysIdDynamic(Direction.kReverse));
    // copilotController.start().and(copilotController.y()).whileTrue(subsystems.getDrivetrain().sysIdQuasistatic(Direction.kForward));
    // copilotController.start().and(copilotController.x()).whileTrue(subsystems.getDrivetrain().sysIdQuasistatic(Direction.kReverse));
  }

  public Subsystems getSubsystems() {
    return this.subsystems;
  }

  public CommandXboxController getDriverController() {
    return driverController;
  }
}
