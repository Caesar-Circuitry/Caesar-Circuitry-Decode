package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
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

    public static double kP = 0.025;
    public static double kI = 0.015;
    public static double kD = 0.0005;
    public static double kS = 0.13;
    public static double Kv = 0.0004065;
    public static double spinupBoost = 0.5;  // Extra power added during spin-up for faster acceleration
    public static double spinupThreshold = 200;  // Switch to normal control when within this velocity of target
    public static double NOMINAL_BATTERY_VOLTAGE = 12;
    public static double VOLTAGE_UPDATE_INTERVAL_SECONDS = 5;

    // Deadband to prevent oscillation when stopping (ticks per second)
    public static final double VELOCITY_DEADBAND = 50.0;
    public static double Detection_DeadBand =20;

    public static double FarVelocity = 1850;
        public static double closeVelocity = 1250;
    public static final double intakeVelocity = -500;

    public static final double LUTDistance = 8.616; // center of the robot to where distance was measured

    // Telemetry logging toggle for Launcher
    public static boolean logTelemetry = true;
  }

  public static class Intake {
    public static final String INTAKE_MOTOR = "Intake";
    public static final String TRANSFER_MOTOR = "Transfer";

    public static final String FEEDER_SERVO = "servo3";

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

    public static final double INTAKE_MOTOR_FORWARD = -1.0;
    public static final double INTAKE_MOTOR_REVERSE = -1.0;
    public static final double INTAKE_MOTOR_HOLD = 0;
    public static final double INTAKE_MOTOR_HP = 0.5;

    public static final double TRANSFER_MOTOR_FORWARD = -1.0;
    public static final double TRANSFER_MOTOR_REVERSE = -1.0;
    public static final double TRANSFER_MOTOR_HOLD = 0;
    public static final double TRANSFER_MOTOR_HP = 0.5;

    // Telemetry logging toggle for Intake
    public static boolean logTelemetry = false;
  }
@Configurable
  public static class Turret {
      public static final String servoName = "servo1";
      public static final String servoName2 = "servo2";
      public static final String servoEncoderName = "rightServo";

      public static final double gearRatio =
              2; // servo rotations per turret rotation (2:1 = servo rotates 2x)
    public static double angleOffset = -180;

      // Large PID - used when error is greater than ERROR_THRESHOLD
      public static double kP_large = 0.012;
      public static double kI_large = 0;
      public static double kD_large = 0.0;

      // Small PID - used when error is less than ERROR_THRESHOLD (fine tuning)
      public static double kP_small = 0.006;
      public static double kI_small = 0;
      public static double kD_small = 0.001;

      // Legacy single PID (kept for compatibility)
      public static  double kP = 0.008;
      public static  double kI = 0;
      public static  double kD = 0.000;

      // Error threshold for switching between large and small PID (in degrees)
      public static double ERROR_THRESHOLD = 30.0;

      public static double kF_left = 0.06; // Feedforward when turning left (positive error)
      public static  double kF_right = -0.08; // Feedforward when turning right (negative error)

      public static  double GoalAngleBlue = -18; // degrees
      public static final double GoalAngleRed = 18;//red
        public static final double WRAP_THRESHOLD = 270.0;

      public static boolean logTelemetry = false;


  }
  public static class Robot {
      public enum motif{
          GPP,
          PGP,
          PPG
      }
      public enum Alliance{
          RED,
          BLUE
      }
      public static motif CurrentMOTIF = motif.GPP; //defaults to green purple purple
      public static Alliance alliance = Alliance.BLUE; //defaults to blue
      public static final Pose BlueGoal = new Pose(6,144-6,0);
      public static final Pose RedGoal = BlueGoal.mirror();
      public static Pose Goal = BlueGoal;
  }
  public static class Vision {
      public static final String cameraName = "limelight";

      // Limelight mounting position on turret (relative to robot center when turret at 0°)
      public static final double LIMELIGHT_X_OFFSET = 0.0; // inches forward from robot center
      public static final double LIMELIGHT_Y_OFFSET = 0.0; // inches left from robot center
      public static final double LIMELIGHT_HEADING_OFFSET = 0.0; // degrees offset from turret angle

      // EKF Tuning parameters - measure these from EKF Tuner
      public static final double X_DRIFT_SIGMA = 0.000348; // inches/sqrt(sec)
      public static final double Y_DRIFT_SIGMA = 0.000999; // inches/sqrt(sec)
      public static final double HEADING_DRIFT_SIGMA = 0.000182; // rad/sqrt(sec)

      public static final double LIMELIGHT_X_STD = 0.001083; // inches
      public static final double LIMELIGHT_Y_STD = 0.000438; // inches
      public static final double LIMELIGHT_HEADING_STD = 0.000437; // radians

      public static final double LOOP_TIME = 0.02; // 20ms = 50Hz

      // Telemetry logging toggle for Vision subsystem
      public static boolean logTelemetry = false;
  }
  public static class Drivetrain {
      public static Pose Pose = new Pose(70, 70, 0);

      // Telemetry logging toggle for Drivetrain
      public static boolean logTelemetry = true;
  }
}
