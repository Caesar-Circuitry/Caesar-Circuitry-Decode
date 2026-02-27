package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
@Disabled
public class DualServoTester extends LinearOpMode {
  private Servo servo;
  private Servo servo2;
  public static String servoName = "";
  public static String servoName2 = "";
  public static double servoPos = 0;
  public static double servoPos2 = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(Servo.class, servoName);
    servo2 = hardwareMap.get(Servo.class, servoName2);
    waitForStart();
    while (opModeIsActive()) {
      if (gamepad1.a) {
        servo.setPosition(servoPos);
      }
      if (gamepad1.b) {
        servo2.setPosition(servoPos2);
      }
      if (gamepad1.y) {
        servo.setPosition(servoPos);
        servo2.setPosition(servoPos2);
      }
    }
  }
}
