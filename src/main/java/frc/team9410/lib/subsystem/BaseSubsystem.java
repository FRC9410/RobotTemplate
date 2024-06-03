package frc.team9410.lib.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.ArrayList;
import java.util.List;

public class BaseSubsystem extends SubsystemBase {
    protected boolean devMode;
    private List<TalonFX> talonFxs;
    private List<CANSparkMax> sparkMaxes;
    protected NetworkTableInstance inst;
    protected NetworkTable subsystemTable;
    protected NetworkTable subsystemPidControllerTable;
    protected NetworkTable dashboardTable;

    public BaseSubsystem() {
        talonFxs = new ArrayList<>();
        sparkMaxes = new ArrayList<>();
        inst = NetworkTableInstance.getDefault();
        dashboardTable = inst.getTable("Dashboard");
        devMode = false;
    }

    // Method to add a TalonFx motor controller
    public void addTalonFx(TalonFX motorController) {
        talonFxs.add(motorController);
    }

    // Method to add a list of TalonFx motor controllers
    public void addTalonFx(List<TalonFX> motorControllers) {
        talonFxs.addAll(motorControllers);
    }

    // Method to add a SparkMax motor controller
    public void addSparkMax(CANSparkMax motorController) {
        sparkMaxes.add(motorController);
    }

    // Method to add a list of SparkMax motor controllers
    public void addSparkMax(List<CANSparkMax> motorControllers) {
        sparkMaxes.addAll(motorControllers);
    }

    // Method to set all motors to a specific speed
    public void setAllMotorSpeed(double speed) {
        for (TalonFX talonFx : talonFxs) {
            talonFx.setControl(new DutyCycleOut(speed));
        }
        for (CANSparkMax sparkMax : sparkMaxes) {
            sparkMax.set(speed);
        }
    }

    // Method to stop all motors
    public void stopAllMotors() {
        for (TalonFX talonFx : talonFxs) {
            talonFx.setControl(new DutyCycleOut(0));
        }
        for (CANSparkMax sparkMax : sparkMaxes) {
            sparkMax.set(0);
        }
    }

    // Method to set all motors to brake mode
    public void setAllMotorsBrakeMode() {
        for (TalonFX talonFx : talonFxs) {
            talonFx.setNeutralMode(NeutralModeValue.Brake);
        }
        for (CANSparkMax sparkMax : sparkMaxes) {
            sparkMax.setIdleMode(IdleMode.kBrake);
        }
    }

    // Method to set all motors to coast mode
    public void setAllMotorsCoastMode() {
        for (TalonFX talonFx : talonFxs) {
            talonFx.setNeutralMode(NeutralModeValue.Coast);
        }
        for (CANSparkMax sparkMax : sparkMaxes) {
            sparkMax.setIdleMode(IdleMode.kCoast);
        }
    }

    public void createSubsystemTable(String name) {
        subsystemTable = inst.getTable(name);
        subsystemPidControllerTable = inst.getTable(name + " PID Controller");
    }

    public void addSparkMaxLogging(CANSparkMax motorController) {
        String deviceName = "CAN ID "+ motorController.getDeviceId();
        subsystemTable.getEntry(deviceName + ": Velocity").setDouble(motorController.getEncoder().getVelocity());
        subsystemTable.getEntry(deviceName + ": Output Current").setDouble(motorController.getOutputCurrent());
        subsystemTable.getEntry(deviceName + ": Output Voltage").setDouble(motorController.getAppliedOutput());
        subsystemTable.getEntry(deviceName + ": Motor Temperature").setDouble(motorController.getMotorTemperature());
        subsystemTable.getEntry("Development Mode").setBoolean(devMode);
    }

    public void setDevMode(boolean devMode) {
        this.devMode = devMode;
    }

    @Override
    public void periodic() {
        setDevMode(dashboardTable.getEntry("Development Mode").getBoolean(false));

        for (CANSparkMax motorController : sparkMaxes) {
            addSparkMaxLogging(motorController);
        }
    }
}
