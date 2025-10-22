package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

public class Constants {
  //    public static class insertSubsystemName {
  // put subsystem specific constants here
  // example: public static final int MOTOR_PORT = 0;
  //    }
  @Configurable
  public static class Launcher {
    public static final double FEED_TIME_SECONDS = 0.20;
    public static final double COOLDOWN_TIME_SECONDS = 0.25;
    public static final double kP = 0.1; // velocity PIDF coefficients for flywheel
    public static final double kI = 0; // velocity PIDF coefficients for flywheel
    public static final double kD = 0; // velocity PIDF coefficients for flywheel
    public static final double kF = 10; // velocity PIDF coefficients for flywheel
    public static final double TARGET_VELOCITY = 1125; // target velocity for flywheel
    public static final double MIN_VELOCITY = 1075;
    public static final double Feeder_STOP_SPEED =
        0.0; // We send this power to the servos when we want them to stop.
    public static final double Feeder_FULL_SPEED = 1.0;
  }
}
