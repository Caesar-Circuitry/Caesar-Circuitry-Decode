package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

public class Constants {
  //    public static class insertSubsystemName {
  // put subsystem specific constants here
  // example: public static final int MOTOR_PORT = 0;
  //    }
  @Configurable
  public static class Launcher {
    // shared launch/feeder constants used across opmodes
    public static final double FEED_TIME_SECONDS = 0.20;
    public static final double INTER_SHOT_PAUSE_SECONDS = 2;
    public static final double COOLDOWN_TIME_SECONDS = 0.25;

    // PIDF / controller gains used in autos & teleop (named to match existing usages)
    public static final double Kp = 0.01;
    public static final double Ki = 0.5;
    public static final double Kd = 0.0;
    public static final double Ks = 0.0431;

    // velocity targets / thresholds
    public static final double TARGET_VELOCITY = 1000;
    public static final double MIN_VELOCITY = 950;

    // feeder servo powers
    public static final double FEEDER_POWER = 1.0;
    public static final double FEEDER_STOP = 0.0;

    // shared shot count used by autos
    public static final int TOTAL_SHOTS = 3;

    // nominal battery voltage used for simple voltage compensation
    public static final double NOMINAL_BATTERY_VOLTAGE = 12.0;
  }
}
