package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.TunerConstants;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public final PIDController forwardPidController =
        new PIDController(DriveConstants.forwardKP, DriveConstants.forwardkI, DriveConstants.forwardkD);
        
    public final PIDController strafePidController =
        new PIDController(DriveConstants.strafeKP, DriveConstants.strafekI, DriveConstants.strafekD);
        
    public final PIDController rotationPidController =
        new PIDController(DriveConstants.rotationKP, DriveConstants.rotationkI, DriveConstants.rotationkD);

    private final SwerveRequest.ApplyChassisSpeeds chassisSpeedRequest =
        new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private Optional<Rotation2d> targetRotation = Optional.empty();
    
    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineRotation;

    // Comment out below requests for CUBE_BOT
    private final SwerveRequest.FieldCentric fieldRelative =
        new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * OIConstants.LEFT_X_DEADBAND)
            .withRotationalDeadband(DriveConstants.MaxAngularRate * OIConstants.RIGHT_X_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotRelative =
        new SwerveRequest.RobotCentric()
            .withDeadband(DriveConstants.MaxSpeed * OIConstants.LEFT_X_DEADBAND)
            .withRotationalDeadband(DriveConstants.MaxAngularRate * OIConstants.RIGHT_X_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants,
        double OdometryUpdateFrequency,
        SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePathPlanner();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    // configurePathPlanner();
    }

    public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
    setControl(requestSupplier.get());
    }

    public void setYaw(double yaw) {
    getPigeon2().setYaw(yaw);
    }

    public void drive(
        double velocityXMetersPerSecond,
        double velocityYMetersPerSecond,
        double rotationRateRadiansPerSecond,
        DriveMode mode) {
        this.targetRotation = Optional.empty();

        switch (mode) {
            case ROBOT_RELATIVE:
            applyRequest(
                () ->
                    robotRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond)
                        .withRotationalRate(-rotationRateRadiansPerSecond));
            break;
            case FIELD_RELATIVE:
            applyRequest(
                () ->
                    fieldRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond)
                        .withRotationalRate(-rotationRateRadiansPerSecond));
            break;
        }
    }

    public void drive(
        double velocityXMetersPerSecond,
        double velocityYMetersPerSecond,
        Optional<Rotation2d> rotationOverride,
        DriveMode mode) {
        this.targetRotation = rotationOverride;

        switch (mode) {
            case ROBOT_RELATIVE:
            applyRequest(
                () ->
                    robotRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond));
            break;
            case FIELD_RELATIVE:
            applyRequest(
                () ->
                    fieldRelative
                        .withVelocityX(-velocityXMetersPerSecond)
                        .withVelocityY(-velocityYMetersPerSecond));
            break;
        }
    }

    /**
     * @return A list of the module positions in the order Front Left, Front Right, Back Left, Back
     *     Right
     */
    public SwerveModulePosition[] getModulePositions() {
        return super.m_modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        return super.getState().ModuleStates;
    }

    public SwerveModuleState[] getModuleTargets() {
        return super.getState().ModuleTargets;
    }

    public Supplier<Pose2d> getPoseSupplier() {
        return new Supplier<Pose2d>() {

            @Override
            public Pose2d get() {
                return getPose();
            }
        };
    }

    private SwerveDriveKinematics getKinematics() {
        return super.m_kinematics;
    }

    public void zeroAll() {
        zeroGyro();
    }

    public void zeroGyro() {
        super.getPigeon2().setYaw(0);
    }

    public void resetPose() {
        setPose(new Pose2d(0, 0, new Rotation2d()));
    }

    public void stopMotorsIntoX() {
        applyRequest(() -> brake);
    }

    public void pointWheels(double degrees) {
        applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(degrees)));
    }

    public void stopMotors() {
        drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
    }

    public Pose2d getPose() {
        return super.m_odometry.getEstimatedPosition();
    }

    public void setPose(Pose2d poseToSet) {
        super.seedFieldRelative(poseToSet);
    }

    public Consumer<ChassisSpeeds> getChassisSpeedsConsumer() {
        return new Consumer<ChassisSpeeds>() {
            @Override
            public void accept(ChassisSpeeds speeds) {
            SwerveModuleState[] moduleStates = getKinematics().toSwerveModuleStates(speeds);
            for (SwerveModuleState state : moduleStates) {
                state.speedMetersPerSecond += DriveConstants.staticKFF;
            }
            setControl(chassisSpeedRequest.withSpeeds(getKinematics().toChassisSpeeds(moduleStates)));
            }
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds chassisSpeeds = getKinematics().toChassisSpeeds(getModuleStates());
        return chassisSpeeds;
    }
    
    public Optional<Rotation2d> getRotationTargetOverride(){
        return targetRotation;
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0.41309;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getChassisSpeeds,
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(3, 0, 0),
                                            new PIDConstants(5, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()-> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public TalonFX[] getMotors() {
        TalonFX[] motors = new TalonFX[8];
        motors[0] = this.Modules[0].getDriveMotor();
        motors[1] = this.Modules[0].getSteerMotor();
        motors[2] = this.Modules[1].getDriveMotor();
        motors[3] = this.Modules[1].getSteerMotor();
        motors[4] = this.Modules[2].getDriveMotor();
        motors[5] = this.Modules[2].getSteerMotor();
        motors[6] = this.Modules[3].getDriveMotor();
        motors[7] = this.Modules[3].getSteerMotor();


        return motors;
    }

    public void configDriveMotors() {
        TalonFX[] motors = this.getMotors();
        TalonFXConfiguration config = new TalonFXConfiguration();
        OpenLoopRampsConfigs openLoopRamps = new OpenLoopRampsConfigs();
        openLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        config.OpenLoopRamps = openLoopRamps;
        
        for(TalonFX motor : motors){
            motor.setNeutralMode(NeutralModeValue.Coast);
            motor.getConfigurator().apply(config);
        }
    }

    public enum DriveMode {
    ROBOT_RELATIVE, FIELD_RELATIVE
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }
}