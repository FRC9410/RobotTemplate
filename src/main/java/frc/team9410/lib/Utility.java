package frc.team9410.lib;

import edu.wpi.first.wpilibj.DriverStation;

public class Utility {

    public static boolean isWithinTolerance(
        double currentValue, double targetValue, double tolerance) {
        return Math.abs(currentValue - targetValue) <= tolerance;
    }

    public static double getSpeed(double speed) {
    double newSpeed = Math.pow(speed, 2);
    return speed > 0 ? newSpeed : -newSpeed;
    // return speed;
    }

    public static String getAllianceColor() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red ? "red" : "blue";
      }
      return "blue";
    }
}