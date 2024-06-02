package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public final String GAME_PIECE_TABLE_NAME = "limelight-game-piece";
    public final String TARGETING_TABLE_NAME = "limelight-targeting";

    public NetworkTable gamePieceTable;
    public NetworkTable targetingTable;

    /** Creates a new Vision. */
    public Vision() {
        gamePieceTable = NetworkTableInstance.getDefault().getTable(GAME_PIECE_TABLE_NAME);
        gamePieceTable.getEntry("ledMode").setNumber(1);

        targetingTable = NetworkTableInstance.getDefault().getTable(TARGETING_TABLE_NAME);
        targetingTable.getEntry("ledMode").setNumber(1);
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

    private NetworkTable getTable(VisionType type) {
        switch (type) {
            case GAME_PIECE:
                return gamePieceTable;
            case TARGET:
                return targetingTable;
            default:
                return null;
        }
    }

    public enum VisionType {
        GAME_PIECE, TARGET
    }
}