package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class ServoTester extends LinearOpMode {
  private Servo servo;
  public static String servoName = "";
  public static double servoPos = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(Servo.class, servoName);
    waitForStart();
    while (opModeIsActive()) {
      if (gamepad1.a) {
        servo.setPosition(servoPos);
      }
    }
  }
}
