package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Configurable
@TeleOp
public class DualContinuousServoTester extends LinearOpMode {
  private CRServo servo;
  private CRServo servo2;
  public static String servoName = "";
  public static String servoName2 = "";

  public enum servoState {
    ONE,
    TWO,
    BOTH
  }

  public servoState state = servoState.ONE;

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(CRServo.class, servoName);
    servo2 = hardwareMap.get(CRServo.class, servoName2);

    waitForStart();
    while (opModeIsActive()) {
      double servopower = 0;
      double servo2power = 0;
      if (gamepad1.a) {
        state = servoState.ONE;
      } else if (gamepad1.y) {
        state = servoState.TWO;
      } else if (gamepad1.b) {
        state = servoState.BOTH;
      }
      switch (state) {
        case ONE:
          servopower = gamepad1.left_stick_x;
          break;
        case TWO:
          servo2power = gamepad1.left_stick_x;
          break;
        case BOTH:
          servopower = gamepad1.left_stick_x;
          servo2power = gamepad1.left_stick_x;
          break;
      }
      servo.setPower(servopower);
      servo2.setPower(servo2power);
      telemetry.addData("s1power", servopower);
      telemetry.addData("s2power", servo2power);
      telemetry.addData("state", state.toString());
      telemetry.update();
    }
  }
}
