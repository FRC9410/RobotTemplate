package frc.robot.subsystems;

import frc.robot.TunerConstants;

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

    // add idle modes
}