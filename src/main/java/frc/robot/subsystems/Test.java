package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Arrays;

import frc.robot.Constants.RobotConstants;
import frc.team9410.lib.subsystem.ControlledSubsystem;

public class Test extends ControlledSubsystem {
  private SparkPIDController pidController;
  private AbsoluteEncoder encoder;

  private TalonFX intake = new TalonFX(10, RobotConstants.kCtreCanBusName);
  CANSparkMax primaryWrist = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(12, MotorType.kBrushless);
  
  private double setpoint;
  
  public Test() {
    super();
    this.primaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.follow(primaryWrist, true);
    
    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    this.encoder.setZeroOffset(0.7);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(5);
    this.pidController.setI(0);
    this.pidController.setD(0);
    this.pidController.setOutputRange(-1, 1);
    
    pidController.setSmartMotionMaxAccel(25000, 0);
    pidController.setSmartMotionMaxVelocity(11000, 0);
    pidController.setSmartMotionAllowedClosedLoopError(0, 0);

    setpoint = 0.11;

    this.pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);

    createSubsystemTable("Test Subystem");
    addAbosultePIDController(pidController, encoder, setpoint);
    addSparkMax(Arrays.asList(primaryWrist, secondaryWrist));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
