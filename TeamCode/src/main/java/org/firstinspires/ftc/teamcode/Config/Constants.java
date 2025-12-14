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

    public static final String FLYWHEEL_MOTOR_LEAD = "FlywheelMotorLead";
    public static final String FLYWHEEL_MOTOR_FOLLOW = "FlywheelMotorFollow";

    public static final boolean FLYWHEEL_MOTOR_LEAD_INVERTED = false;
    public static final boolean FLYWHEEL_MOTOR_FOLLOW_INVERTED = true;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
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

    public static final double FEEDER_SERVO_OPEN = 0.0;
    public static final double FEEDER_SERVO_CLOSE = 0.0;

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
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;

    public static final double GoalAngle = 144; // degrees
    public static final double minAngle = -135; // degrees
    public static final double maxAngle = 135; // degrees

    public static final double TURRET_KF_MODEL_COVARIANCE = 0.01;
    // How much we trust the model (Pinpoint robot heading)
    // Smaller = trust odometry more

    public static final double TURRET_KF_DATA_COVARIANCE = 6.0;
    // How noisy the Limelight tx measurement is (deg^2)
    // Higher = trust Limelight less

    public static final double TURRET_KF_START_STATE = 0.0;
    // Initial turret relative angle (deg)

    public static final double TURRET_KF_START_VARIANCE = 16.0;
    // Initial uncertainty (~4 deg standard deviation)

    public static final double TURRET_KF_START_GAIN = 0.5;
    // Initial Kalman gain (0–1)

  }
}
