package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

public class Music extends SubsystemBase {
  Orchestra orchestra;

  public Music(CommandSwerveDrivetrain drivetrain) {

    orchestra = new Orchestra();
    for(TalonFX motor : drivetrain.getMotors()){
      orchestra.addInstrument(motor);
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void playSong(String songName) {
    String deployPath = Filesystem.getDeployDirectory().getAbsolutePath();
    boolean playingMusic = true;
    orchestra.loadMusic(deployPath + "/" + songName + "Output.chrp");
    orchestra.play();
    while(playingMusic) {
      if (!orchestra.isPlaying()) {
        playingMusic = false;
      }
    }
    orchestra.close();
  }
}