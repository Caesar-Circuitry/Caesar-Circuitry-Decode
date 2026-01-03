package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
  //    public static class insertSubsystemName {
  // put subsystem specific constants here
  // example: public static final int MOTOR_PORT = 0;
  //    }

  public static class intake {
      public static final String INTAKE_MOTOR = "intake";
      public static final String TRANSFER_MOTOR = "transfer";

      public static final String FEEDER_SERVO = "block";

      public static final String intakeBeamBreak = "intakeBeam";
      public static final String transferBeamBreak = "transferBeam";

      public static final double intakeOverCurrent = 6;
      public static final double transferOverCurrent = 6;

      public static final DcMotorSimple.Direction INTAKE_MOTOR_DIRECTION =
              DcMotorSimple.Direction.FORWARD;
      public static final DcMotorSimple.Direction TRANSFER_MOTOR_DIRECTION =
              DcMotorSimple.Direction.FORWARD;

      public static final double FEEDER_SERVO_OPEN = 0.0;
      public static final double FEEDER_SERVO_CLOSE = 0.0;

      public static final double INTAKE_MOTOR_FORWARD = 1.0;
      public static final double INTAKE_MOTOR_HOLD = 0;
      public static final double INTAKE_MOTOR_HP = -1.0;

      public static final double TRANSFER_MOTOR_FORWARD = 1.0;
      public static final double TRANSFER_MOTOR_HOLD = 0;
      public static final double TRANSFER_MOTOR_HP = -1.0;
  }
}
