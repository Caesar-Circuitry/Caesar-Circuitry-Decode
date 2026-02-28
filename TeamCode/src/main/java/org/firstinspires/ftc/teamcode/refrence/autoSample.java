package org.firstinspires.ftc.teamcode.refrence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class autoSample extends LinearOpMode {
  // put class members here
  @Override
  public void runOpMode() throws InterruptedException {
    // initialization code here
    waitForStart();
    while (opModeIsActive()) {
      // code to run repeatedly during teleop
    }
  }
}
