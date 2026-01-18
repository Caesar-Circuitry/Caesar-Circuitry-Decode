package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
  //    public static class insertSubsystemName {
  // put subsystem specific constants here
  // example: public static final int MOTOR_PORT = 0;
  //    }
  @Configurable
  public static class Launcher {

    public static final String FLYWHEEL_MOTOR_LEAD = "Flywheel1";
    public static final String FLYWHEEL_MOTOR_FOLLOW = "Flywheel2";

    public static final boolean FLYWHEEL_MOTOR_LEAD_INVERTED = false;
    public static final boolean FLYWHEEL_MOTOR_FOLLOW_INVERTED = true;

    public static final double kP = 0.01;
    public static final double kI = 0.5;
    public static final double kD = 0.0;
    public static final double kS = 0.13;
    public static final double NOMINAL_BATTERY_VOLTAGE = 12;
    public static final double VOLTAGE_UPDATE_INTERVAL_SECONDS = 5;

    // Deadband to prevent oscillation when stopping (ticks per second)
    public static final double VELOCITY_DEADBAND = 50.0;

    public static final double FarVelocity = 1500;
    public static final double closeVelocity = 1000;
    public static final double intakeVelocity = -500;
  }

  @Configurable
  public static class Intake {
    public static final String INTAKE_MOTOR = "IntakeMotor";
    public static final String TRANSFER_MOTOR = "TransferMotor";

    public static final String FEEDER_SERVO = "FeederServo";

    public static final String intakeBeamBreak = "intakeBeamBreak";
    public static final String transferBeamBreak = "transferBeamBreak";

    public static final double intakeOverCurrent = 6;
    public static final double transferOverCurrent = 6;

    public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION =
        DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER_MOTOR_DIRECTION =
        DcMotorSimple.Direction.FORWARD;

    public static final double FEEDER_SERVO_OPEN = 0.5;
    public static final double FEEDER_SERVO_CLOSE = 0.63;

    public static final double INTAKE_MOTOR_FORWARD = 1.0;
    public static final double INTAKE_MOTOR_REVERSE = -1.0;
    public static final double INTAKE_MOTOR_HOLD = 0;
    public static final double INTAKE_MOTOR_HP = -1.0;

    public static final double TRANSFER_MOTOR_FORWARD = 1.0;
    public static final double TRANSFER_MOTOR_REVERSE = -1.0;
    public static final double TRANSFER_MOTOR_HOLD = 0;
    public static final double TRANSFER_MOTOR_HP = -1.0;
  }

  @Configurable
  public static class Turret {
      public static String servoName = "servo1";
      public static String servoName2 = "servo2";
      public static String servoEncoderName = "rightServo";

      public static double gearRatio =
              2; // servo rotations per turret rotation (2:1 = servo rotates 2x)

      public static double kP = 0.008;
      public static double kI = 0;
      public static double kD = 0;
      public static double kF_left = 0.07; // Feedforward when turning left (positive error)
      public static double kF_right = -0.1; // Feedforward when turning right (negative error)

      public static final double GoalAngle = 144; // degrees
        public static final double WRAP_THRESHOLD = 270.0;

  }
}
