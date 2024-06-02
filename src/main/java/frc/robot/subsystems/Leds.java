package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class Leds extends SubsystemBase {
    public final int CANDLE_ID = 23;
    public final String CANDLE_CAN_BUS = "rio";

    private CANdle candle;

    /** Creates a new Leds. */
    public Leds() {
        candle = new CANdle(CANDLE_ID, CANDLE_CAN_BUS);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
    }

    public void setFadeAnimtation(int r, int g, int b) {
        candle.animate(new SingleFadeAnimation(r, g, b, 0, 0.75, 8)); 
    }

    public void setFireAnimtation() {
        candle.animate(new FireAnimation(0.5, 0.7, 8, 0.7, 0.5));
    }

    public void setRainbowAnimtation() {
        candle.animate(new RainbowAnimation(1, 0.1, 8));
    }

    public void setRgbFadeAnimtation() {
        candle.animate(new RgbFadeAnimation(0.7, 0.4, 8));
    }

    public void setStrobeAnimtation(int r, int g, int b) {
        candle.animate(new StrobeAnimation(r, g, b, 0, 98.0 / 256.0, 8));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}