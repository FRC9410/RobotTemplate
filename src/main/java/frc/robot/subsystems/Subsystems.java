package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import frc.team9410.lib.LimelightHelpers;

import java.util.Map;

/** Add your docs here. */
public class Subsystems {
    private CommandSwerveDrivetrain drivetrain;
    private Leds leds;
    private Music music;
    private Vision vision;
    private RobotState robotState;
    private Test test;

    public Subsystems(CommandXboxController controller) {
        this.drivetrain = TunerConstants.DriveTrain;
        this.leds = new Leds();
        this.music = new Music(this.drivetrain);
        this.vision = new Vision();
        this.robotState = new RobotState(this.drivetrain, this.vision, controller);

        this.test = new Test();
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public Leds getLeds() {
        return leds;
    }

    public Music getMusic() {
        return music;
    }

    public Vision getVision() {
        return vision;
    }

    public Test getTest() {
        return test;
    }

    public void setDisabledIdleMode() {

    }

    public void setEnabledIdleMode() {

    }

    public void updatePosition() {
        Map<String, Object> poseWitTimeEstimate = vision.getPoseEstimate(drivetrain.getPose().getRotation().getDegrees());
        if (poseWitTimeEstimate != null) {
            Pose2d pose = (Pose2d) poseWitTimeEstimate.get("pose");
            pose.rotateBy(Rotation2d.fromDegrees(180));
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(
                pose,
                (double) poseWitTimeEstimate.get("timestamp")
            );
            drivetrain.seedFieldRelative(pose);
        }
    }
}