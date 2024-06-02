package frc.team9410.lib.subsystem;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;

public class ControlledSubsystem extends BaseSubsystem {
    private SparkPIDController pidController;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;

    public ControlledSubsystem() {
        super();
    }

    public void addAbosultePIDController(SparkPIDController pidController, AbsoluteEncoder encoder) {
        this.pidController = pidController;
        this.absoluteEncoder = encoder;
    }

    public void addRelativePIDController(SparkPIDController pidController, RelativeEncoder encoder) {
        this.pidController = pidController;
        this.relativeEncoder = encoder;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        // This method will be called once per scheduler run
        // Common periodic actions can be defined here
    }
}
