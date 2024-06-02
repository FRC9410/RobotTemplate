package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.TunerConstants;
import frc.robot.utils.Utility;

import java.util.Map;

/** Add your docs here. */
public class Subsystems {
    private CommandSwerveDrivetrain drivetrain;
    private Leds leds;
    private Music music;
    private Vision vision;
    private Test test;

    public Subsystems() {
        this.drivetrain = TunerConstants.DriveTrain;
        this.leds = new Leds();
        this.music = new Music(this.drivetrain);
        this.vision = new Vision();

        this.test = new Test();
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
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

    public void updatePosition(PeriodType periodType) {
        String allianceColor = periodType == PeriodType.AUTO || Utility.getAllianceColor() == "blue" ? "blue" : "red";
        Map<String, Object> poseWitTimeEstimate = vision.getPoseEstimate(allianceColor);
        if (poseWitTimeEstimate != null) {
            Pose3d pose = (Pose3d) poseWitTimeEstimate.get("pose");
            Pose2d newPose = pose.toPose2d();
            newPose.rotateBy(Rotation2d.fromDegrees(180));
            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            drivetrain.addVisionMeasurement(
                newPose,
                (double) poseWitTimeEstimate.get("timestamp")
            );
            drivetrain.seedFieldRelative(newPose);
        }
    }

    public enum PeriodType {
        AUTO, TELEOP
    }
}