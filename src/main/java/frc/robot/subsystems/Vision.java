package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team9410.lib.LimelightHelpers;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class Vision extends SubsystemBase {
    public final String GAME_PIECE_TABLE_NAME = "limelight-game-piece";
    public final String TAG_TABLE_NAME = "limelight-targeting";

    public NetworkTable gamePieceTable;
    public NetworkTable tagTable;

    /** Creates a new Vision. */
    public Vision() {
        gamePieceTable = NetworkTableInstance.getDefault().getTable(GAME_PIECE_TABLE_NAME);
        gamePieceTable.getEntry("ledMode").setNumber(1);

        tagTable = NetworkTableInstance.getDefault().getTable(TAG_TABLE_NAME);
        tagTable.getEntry("ledMode").setNumber(1);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public double getTx(VisionType type) {
        return getTable(type).getEntry("tx").getDouble(0);
    }

    public double getTy(VisionType type) {
        return  getTable(type).getEntry("ty").getDouble(0);
    }

    public double getTa(VisionType type) {
        return getTable(type).getEntry("ta").getDouble(0);
    }

    public int getTagId(VisionType type) {
        double tagId = getTable(type).getEntry("tid").getDouble(0);
        return (int) tagId;
    }

    public boolean hasTarget(VisionType type) {
        return getTable(type).getEntry("tv").getDouble(0) == 1;
    }

    public void setPipeline(VisionType type, int pipeline) {
        getTable(type).getEntry("pipeline").setNumber(pipeline);
    }
  
    public int getPipeline(VisionType type) {
        return (int) getTable(type).getEntry("getpipe").getDouble(-1);
    }

    public Map<String, Object> getPoseEstimate (double yaw) {
        LimelightHelpers.SetRobotOrientation(TAG_TABLE_NAME, yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(TAG_TABLE_NAME);
        if(limelightMeasurement == null) {
            return null;
        }
        Pose2d pose = limelightMeasurement.pose;
            Map<String, Object> result = new HashMap<>();
            result.put("pose", pose);
            result.put("timestamp", limelightMeasurement.timestampSeconds);
            return result;
    }

    public Optional<Rotation2d> getGamePieceRotation() {
        if(!hasTarget(VisionType.GAME_PIECE)) {
            return Optional.empty();
        }
        return Optional.of(Rotation2d.fromDegrees(getTx(VisionType.GAME_PIECE)));
    }

    private NetworkTable getTable(VisionType type) {
        switch (type) {
            case GAME_PIECE:
                return gamePieceTable;
            case TAG:
                return tagTable;
            default:
                return null;
        }
    }

    public enum VisionType {
        GAME_PIECE, TAG
    }
}