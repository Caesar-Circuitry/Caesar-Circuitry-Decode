package org.firstinspires.ftc.teamcode.Opmodes.Tester;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class DualServoTesterPinpoint extends LinearOpMode {
  private Servo servo;
  private Servo servo2;
  private GoBildaPinpointDriver pinpoint;
  public static String servoName = "";
  public static String servoName2 = "";
  public static double targetAngle = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    servo = hardwareMap.get(Servo.class, servoName);
    servo2 = hardwareMap.get(Servo.class, servoName2);
    pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    // Configure the sensor
    configurePinpoint();

    // Set the location of the robot - this should be the place you are starting the robot from
    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    waitForStart();
    while (opModeIsActive()) {
      pinpoint.update();
      double heading = pinpoint.getHeading(AngleUnit.DEGREES);
      double rawError = targetAngle - heading;
      double wrappedError = wrap180(rawError);
      double clampedError = clamp(wrappedError, -90.0, 90.0);
      double servoAngle = clampedError;
      double servoPos = (servoAngle + 90.0) / 180.0;
      servoPos = clamp(servoPos, 0.0, 1.0);
      servo.setPosition(servoPos);
      servo2.setPosition(servoPos);
      telemetry.addData("heading", heading);
      telemetry.update();
    }
  }

  public void configurePinpoint() {
    /*
     *  Set the odometry pod positions relative to the point that you want the position to be measured from.
     *
     *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
     *  Left of the center is a positive number, right of center is a negative number.
     *
     *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
     *  Forward of center is a positive number, backwards is a negative number.
     */
    pinpoint.setOffsets(
        -84.0, -168.0, DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1

    /*
     * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
     * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
     * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
     * number of ticks per unit of your odometry pod.  For example:
     *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
     */
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    /*
     * Set the direction that each of the two odometry pods count. The X (forward) pod should
     * increase when you move the robot forward. And the Y (strafe) pod should increase when
     * you move the robot to the left.
     */
    pinpoint.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /*
     * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
     * The IMU will automatically calibrate when first powered on, but recalibrating before running
     * the robot is a good idea to ensure that the calibration is "good".
     * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
     * This is recommended before you run your autonomous, as a bad initial calibration can cause
     * an incorrect starting value for x, y, and heading.
     */
    pinpoint.resetPosAndIMU();
  }

  private double wrap180(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
  }

  private double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }
}
