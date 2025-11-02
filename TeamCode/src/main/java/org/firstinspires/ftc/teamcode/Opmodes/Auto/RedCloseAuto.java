package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

@Autonomous(name = "RedCloseAuto")
public class RedCloseAuto extends OpMode {
  private Follower follower;
  // use ElapsedTime instead of Pedro Timer
  private ElapsedTime pathTimer, opmodeTimer, interShotTimer;
  private int pathState;

  // Mirrored values from BlueCloseAuto: x' = 144 - x, heading' = PI - heading
  private final Pose startPose =
      new Pose(80.308, 135.771, Math.PI - Math.toRadians(90)); // mirrored start
  private final Pose scorePose =
      new Pose(118.0, 126, Math.PI - Math.toRadians(146)); // mirrored score

  // post-shot path to mirror BlueCloseAuto behavior
  private Path postShotPath;

  // Launcher hardware and shooting state
  private DcMotorEx launcher;
  private CRServo leftFeeder, rightFeeder;
  private ElapsedTime feederTimer;
  private int shotsFired = 0;

  // LAUNCHER_OPEN_LOOP_POWER remains here as it's specific to this opmode
  private static final double LAUNCHER_OPEN_LOOP_POWER = 0.85; // open-loop spin power (fallback)

  // PIDF-based launcher control (ported from StarterBotTeleopMecanums)
  private PIDFController launchController;
  // PID values taken from constants
  private double actualVelocity = 0;
  private double LAUNCHER_DESIRED_VELOCITY = 0;

  // Battery voltage compensation
  private List<VoltageSensor> voltageSensors;
  private double batteryVoltage = Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
  private ElapsedTime voltageTimer;

  public void buildPaths() {
    postShotPath = new Path(new BezierCurve(
            new Pose(118.000, 126.000), // start
            new Pose(100.000, 128.500), // control point
            new Pose(82.000, 131.000)   // end
    ));
    postShotPath.setLinearHeadingInterpolation(Math.toRadians(34), Math.toRadians(35));
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        // start following the built path
        setPathState(1);
        break;
      case 1:
        // Wait until follower completes, then begin shooting routine
        if (!follower.isBusy()) {
          setPathState(2); // begin spin-up
        }
        break;
      case 2:
        // Spin up launcher using closed-loop PIDF to the desired velocity
        LAUNCHER_DESIRED_VELOCITY = Constants.Launcher.TARGET_VELOCITY;
        // Feed immediately once the launcher reports being at-or-above the minimum velocity
        if (launcher != null
            && Math.abs(launcher.getVelocity()) >= Constants.Launcher.MIN_VELOCITY) {
          feederTimer.reset();
          leftFeeder.setPower(Constants.Launcher.FEEDER_POWER);
          rightFeeder.setPower(Constants.Launcher.FEEDER_POWER);
          setPathState(3);
        }
        break;
      case 3:
        // Feeding - run feeders for FEED_TIME_SECONDS, then stop and count a shot
        if (feederTimer.seconds() >= Constants.Launcher.FEED_TIME_SECONDS) {
          leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
          rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
          shotsFired++;
          if (shotsFired >= Constants.Launcher.TOTAL_SHOTS) {
            // done shooting
            LAUNCHER_DESIRED_VELOCITY = 0;
            if (launcher != null) launcher.setPower(0);
            // start the requested post-shot path (mirrored) if available
            if (postShotPath != null && follower != null) {
              follower.followPath(postShotPath);
            }
            setPathState(4);
          } else {
            // start inter-shot pause to allow flywheel to spin up again before next feed
            if (interShotTimer != null) interShotTimer.reset();
            setPathState(5);
          }
        }
        break;
      case 4:
        // finished/post-shot: idle or waiting for follower to complete
        break;
      case 5:
        // Inter-shot pause: wait for configured seconds before trying to spin-up and feed again
        if (interShotTimer != null
            && interShotTimer.seconds() >= Constants.Launcher.INTER_SHOT_PAUSE_SECONDS) {
          setPathState(2); // go back to spin-up (closed-loop) and then feed when ready
        }
        break;
      default:
        break;
    }
  }

  /**
   * These change the states of the paths and actions. It will also reset the timers of the
   * individual switches *
   */
  public void setPathState(int pState) {
    pathState = pState;
    if (pathTimer != null) pathTimer.reset();
  }

  /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". * */
  @Override
  public void loop() {

    // These loop the movements of the robot, these must be called continuously in order to work
    if (follower != null) follower.update();
    autonomousPathUpdate();

    // Update battery voltage periodically (small overhead)
    if (voltageTimer != null && voltageTimer.seconds() >= 1.0) {
      batteryVoltage = getBatteryVoltage();
      voltageTimer.reset();
    }

    // Closed-loop launcher velocity control (if controller present)
    if (launcher != null && launchController != null) {
      actualVelocity = launcher.getVelocity();
      if (!(LAUNCHER_DESIRED_VELOCITY == 0)) {
        double power =
            (MathUtils.clamp(
                        (launchController.calculate(actualVelocity, LAUNCHER_DESIRED_VELOCITY)
                            + Constants.Launcher.Ks),
                        -1,
                        1)
                    * Constants.Launcher.NOMINAL_BATTERY_VOLTAGE)
                / batteryVoltage;
        launcher.setPower(power);
      } else {
        launcher.setPower(0);
      }
    }

    // Feedback to Driver Hub for debugging
    telemetry.addData("path state", pathState);
    if (follower != null && follower.getPose() != null) {
      telemetry.addData("x", follower.getPose().getX());
      telemetry.addData("y", follower.getPose().getY());
      telemetry.addData("heading", follower.getPose().getHeading());
    }
    if (launcher != null) telemetry.addData("launcherVel", launcher.getVelocity());
    telemetry.addData("shotsFired", shotsFired);
    telemetry.addData("launcherDesired", LAUNCHER_DESIRED_VELOCITY);
    telemetry.addData("batteryV", batteryVoltage);
    telemetry.update();
  }

  /** This method is called once at the init of the OpMode. * */
  @Override
  public void init() {
    // use ElapsedTime for all timers
    pathTimer = new ElapsedTime();
    opmodeTimer = new ElapsedTime();
    interShotTimer = new ElapsedTime();
    feederTimer = new ElapsedTime();
    voltageTimer = new ElapsedTime();
    opmodeTimer.reset();

    follower = PedroConstants.createFollower(hardwareMap);
    buildPaths();
    follower.setStartingPose(startPose);

    // Map launcher hardware (use same names as TeleOp)
    try {
      launcher = hardwareMap.get(DcMotorEx.class, "launcher");
      leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
      rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

      // configure launcher encoder mode and zero power behavior
      launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      // feeders initial state
      leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
      rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
      leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

      // initialize PIDF controller and voltage sensors for closed-loop control
      launchController =
          new PIDFController(
              Constants.Launcher.Kp, Constants.Launcher.Ki, Constants.Launcher.Kd, 0);
      voltageSensors = hardwareMap.getAll(VoltageSensor.class);
      batteryVoltage = getBatteryVoltage();
    } catch (Exception e) {
      telemetry.addData("Launcher init error", e.getMessage());
    }

    // reset shot counter
    shotsFired = 0;
    pathState = -1; // don't start until play
  }

  /** This method is called continuously after Init while waiting for "play". * */
  @Override
  public void init_loop() {}

  /**
   * This method is called once at the start of the OpMode. It runs all the setup actions, including
   * building paths and starting the path system *
   */
  @Override
  public void start() {
    opmodeTimer.reset();
    // begin autonomous path + shoot sequence
    setPathState(0);
  }

  /** We do not use this because everything should automatically disable * */
  @Override
  public void stop() {
    if (launcher != null) launcher.setPower(0);
    if (leftFeeder != null) leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
    if (rightFeeder != null) rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
  }

  // Helper: read battery voltage (copied from TeleOp helper)
  private double getBatteryVoltage() {
    if (voltageSensors == null || voltageSensors.isEmpty())
      return Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
    double minV = Double.POSITIVE_INFINITY;
    for (VoltageSensor vs : voltageSensors) {
      double v = vs.getVoltage();
      if (v > 0) minV = Math.min(minV, v);
    }
    if (minV == Double.POSITIVE_INFINITY) {
      return Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
    }
    return minV;
  }
}
