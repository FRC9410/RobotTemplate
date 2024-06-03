package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class Dashboard {
    private boolean devMode;
    private NetworkTableInstance inst;
    private NetworkTable dashboardTable;
    private SendableChooser<Command> autoChooser;

    public Dashboard() {
        inst = NetworkTableInstance.getDefault();
        dashboardTable = inst.getTable("Dashboard");
        devMode = false;
    }

    public void loadDashboard(Subsystems subsystems) {
        loadAutoChooser();
        dashboardTable.getEntry("Development Mode").setBoolean(devMode);
        // dashboardTable.getEntry("Subsystems").setString(subsystems.toString());
    }

    public void updateDashboard(Subsystems subsystems) {
        devMode = dashboardTable.getEntry("Development Mode").getBoolean(false);
        // dashboardTable.getEntry("Subsystems").setString(subsystems.toString());
    }

    public void loadAutoChooser() {
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("deploy/autos");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void registerNamedCommands() {
      // NamedCommands.registerCommand("TestCommand", new TestCommand(subsystems));
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
