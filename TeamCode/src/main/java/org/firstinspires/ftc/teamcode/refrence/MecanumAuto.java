package org.firstinspires.ftc.teamcode.refrence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "MecanumAuto", group = "reference")
public class MecanumAuto extends LinearOpMode {
  // Drive motors (expected hardware names: frontLeft, frontRight, backLeft, backRight)
  private DcMotor frontLeft;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor backRight;

  // Tunable constants
  private static final double DRIVE_POWER = 0.5; // forward power (0.0 - 1.0)
  private static final double DRIVE_MS = 1000.0; // drive duration in milliseconds (forward/back)
  private static final double PAUSE_MS = 250.0; // short pause between maneuvers

  @Override
  public void runOpMode() throws InterruptedException {

    // Map hardware
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    backRight = hardwareMap.get(DcMotor.class, "backRight");

    // Motor direction: adjust if your robot moves opposite
    frontLeft.setDirection(DcMotor.Direction.FORWARD);
    backLeft.setDirection(DcMotor.Direction.FORWARD);
    frontRight.setDirection(DcMotor.Direction.REVERSE);
    backRight.setDirection(DcMotor.Direction.REVERSE);

    waitForStart(); // wait for the play button

    if (isStopRequested()) return;

    // Use ElapsedTime and an explicit state variable rather than sleeping
    ElapsedTime timer = new ElapsedTime();
    int state = 1; // 1 = drive forward, 2 = pause, 3 = drive backward, 4 = done
    timer.reset();

    while (opModeIsActive() && state != 4) {
      if (state == 1) {
        // start driving forward
        setAllPower(DRIVE_POWER);
        if (timer.milliseconds() >= DRIVE_MS) {
          stopDrive();
          state = 2;
          timer.reset();
        }
      } else if (state == 2) {
        // pause
        if (timer.milliseconds() >= PAUSE_MS) {
          state = 3;
          timer.reset();
        }
      } else if (state == 3) {
        // drive backward
        setAllPower(-DRIVE_POWER);
        if (timer.milliseconds() >= DRIVE_MS) {
          stopDrive();
          state = 4; // finished
        }
      }

      // Telemetry for debugging
      telemetry.addData("State", state);
      telemetry.addData("Timer(ms)", timer.milliseconds());
      telemetry.update();

      idle(); // let the system do background work
    }

    // Ensure motors are stopped at end
    stopDrive();

    telemetry.addData("Status", "Finished");
    telemetry.update();
  }

  // Helper: set power to all drive motors
  private void setAllPower(double power) {
    frontLeft.setPower(power);
    frontRight.setPower(power);
    backLeft.setPower(power);
    backRight.setPower(power);
  }

  // Helper: stop all drive motors
  private void stopDrive() {
    setAllPower(0.0);
  }
}
