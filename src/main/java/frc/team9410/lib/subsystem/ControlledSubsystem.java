package frc.team9410.lib.subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ControlledSubsystem extends BaseSubsystem {
    private SparkPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    private double setpoint;

    public ControlledSubsystem() {
        super();
    }

    public void addAbosultePIDController(SparkPIDController pidController, AbsoluteEncoder encoder, double setpoint) {
        this.pidController = pidController;
        this.absoluteEncoder = encoder;
        this.setpoint = setpoint;
    }

    public void addRelativePIDController(SparkPIDController pidController, RelativeEncoder encoder, double setpoint) {
        this.pidController = pidController;
        this.relativeEncoder = encoder;
        this.setpoint = setpoint;
    }

    public void addSparkMaxPidControllerLogging() {
        subsystemTable.getEntry("Setpoint").setDouble(setpoint);
        subsystemTable.getEntry("kP").setDouble(pidController.getP());
        subsystemTable.getEntry("kI").setDouble(pidController.getI());
        subsystemTable.getEntry("kD").setDouble(pidController.getD());
        subsystemTable.getEntry("kFF").setDouble(pidController.getFF());

        if (absoluteEncoder != null || relativeEncoder != null) {
            subsystemTable.getEntry("Encoder Position").setDouble(absoluteEncoder.getPosition());
        }
    }
    
    @Override
    public void periodic() {
        super.periodic();
        if (pidController != null) {
            addSparkMaxPidControllerLogging();
        }
    }
}
