package frc.team9410.lib.subsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ControlledSubsystem extends BaseSubsystem {
    private SparkPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    private boolean velocityControlled;
    private boolean shouldUpdate;
    private double setpoint;
    private double newSetpoint;
    private double newkP;
    private double newkI;
    private double newkD;
    private double newkFF;

    public ControlledSubsystem() {
        super();
        shouldUpdate = false;
    }

    public void addAbosultePIDController(SparkPIDController pidController, AbsoluteEncoder encoder, double setpoint) {
        this.pidController = pidController;
        this.absoluteEncoder = encoder;
        this.setpoint = setpoint;
        this.velocityControlled = false;
    }

    public void addAbosultePIDController(SparkPIDController pidController, AbsoluteEncoder encoder, double setpoint, boolean velocityControlled) {
        this.pidController = pidController;
        this.absoluteEncoder = encoder;
        this.setpoint = setpoint;
        this.velocityControlled = velocityControlled;
    }

    public void addRelativePIDController(SparkPIDController pidController, RelativeEncoder encoder, double setpoint) {
        this.pidController = pidController;
        this.relativeEncoder = encoder;
        this.setpoint = setpoint;
        this.velocityControlled = false;
    }

    public void addRelativePIDController(SparkPIDController pidController, RelativeEncoder encoder, double setpoint, boolean velocityControlled) {
        this.pidController = pidController;
        this.relativeEncoder = encoder;
        this.setpoint = setpoint;
        this.velocityControlled = velocityControlled;
    }

    public void addSparkMaxPidControllerLogging() {
        subsystemTable.getEntry("Setpoint").setDouble(getEditValue("Setpoint"));
        subsystemTable.getEntry("kP").setDouble(getEditValue("kP"));
        subsystemTable.getEntry("kI").setDouble(getEditValue("kI"));
        subsystemTable.getEntry("kD").setDouble(getEditValue("kD"));
        subsystemTable.getEntry("kFF").setDouble(getEditValue("kFF"));
        subsystemTable.getEntry("Update").setBoolean(getBooleanEditValue("Update"));
        
        subsystemPidControllerTable.getEntry("Controller Setpoint").setDouble(setpoint);
        subsystemPidControllerTable.getEntry("Controller kP").setDouble(pidController.getP());
        subsystemPidControllerTable.getEntry("Controller kI").setDouble(pidController.getI());
        subsystemPidControllerTable.getEntry("Controller kD").setDouble(pidController.getD());
        subsystemPidControllerTable.getEntry("Controller kFF").setDouble(pidController.getFF());

        if (absoluteEncoder != null || relativeEncoder != null) {
            subsystemTable.getEntry("Encoder Position").setDouble(absoluteEncoder.getPosition());
        }

        if (shouldUpdate) {
            pidController.setP(newkP);
            pidController.setI(newkI);
            pidController.setD(newkD);
            pidController.setFF(newkFF);
            pidController.setReference(newSetpoint, velocityControlled ? CANSparkMax.ControlType.kVelocity : CANSparkMax.ControlType.kPosition);
            setpoint = newSetpoint;
            shouldUpdate = false;
            subsystemTable.getEntry("Update").setBoolean(false);
        }
    }

    public double getEditValue(String key) {
        double currentValue = subsystemTable.getEntry(key).getDouble(0);
        if (devMode) {
            switch (key) {
                case "kP":
                    newkP = currentValue;
                    break;
                case "kI":
                    newkI = currentValue;
                    break;
                case "kD":
                    newkD = currentValue;
                    break;
                case "kFF":
                    newkFF = currentValue;
                    break;
                case "Setpoint":
                    newSetpoint = currentValue;
                    break;
                default:
                    break;
            }
            return currentValue;
        }

        switch (key) {
            case "kP":
                newkP = pidController.getP();
                return newkP;
            case "kI":
                newkI = pidController.getI();
                return newkI;
            case "kD":  
                newkD = pidController.getD();
                return newkD;
            case "kFF":
                newkFF = pidController.getFF();
                return newkFF;
            case "Setpoint":
                newSetpoint = setpoint;
                return newSetpoint;
            default:
                return 0.0;
        }
    }

    public boolean getBooleanEditValue(String key) {
        boolean currentValue = subsystemTable.getEntry(key).getBoolean(false);
        if (devMode) {
            shouldUpdate = currentValue;
            return currentValue;
        }
        return false;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        if (pidController != null) {
            addSparkMaxPidControllerLogging();
        }
    }
}
