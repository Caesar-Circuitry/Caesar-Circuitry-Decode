package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.Config.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
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
  private Timer pathTimer, opmodeTimer;
  private int pathState;

  // Values taken from Config/paths/RedSideClose.pp
  private final Pose startPose = new Pose(129, 112, Math.toRadians(90));
  private final Pose scorePose = new Pose(120, 127, Math.toRadians(38));

  private Path scorePath;

  // Launcher hardware and shooting state
  private DcMotorEx launcher;
  private CRServo leftFeeder, rightFeeder;
  private ElapsedTime feederTimer;
  private int shotsFired = 0;

  // PIDF-based launcher control
  private PIDFController launchController;
  private double actualVelocity = 0;
  private double LAUNCHER_DESIRED_VELOCITY = 0;

  // Battery voltage compensation
  private List<VoltageSensor> voltageSensors;
  private static final double NOMINAL_BATTERY_VOLTAGE = 12.0;
  private double batteryVoltage = NOMINAL_BATTERY_VOLTAGE;
  private ElapsedTime voltageTimer;

  public void buildPaths() {
    // control point from RedSideClose.pp: (117,113)
    scorePath = new Path(new BezierCurve(startPose, new Pose(117, 113, 0), scorePose));
    scorePath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        // start following the built path
        follower.followPath(scorePath);
        setPathState(1);
        break;
      case 1:
        // Wait until follower completes, then begin shooting routine
        if (!follower.isBusy()) {
          setPathState(2);
        }
        break;
      case 2:
        // Spin up launcher using closed-loop
        LAUNCHER_DESIRED_VELOCITY = Constants.Launcher.TARGET_VELOCITY;
        if (launcher != null && Math.abs(launcher.getVelocity()) >= Constants.Launcher.MIN_VELOCITY) {
          feederTimer.reset();
          leftFeeder.setPower(Constants.Launcher.FEEDER_POWER);
          rightFeeder.setPower(Constants.Launcher.FEEDER_POWER);
          setPathState(3);
        }
        break;
      case 3:
        if (feederTimer.seconds() >= Constants.Launcher.FEED_TIME_SECONDS) {
          leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
          rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
          shotsFired++;
          if (shotsFired >= Constants.Launcher.TOTAL_SHOTS) {
            LAUNCHER_DESIRED_VELOCITY = 0;
            if (launcher != null) launcher.setPower(0);
            setPathState(4);
          } else {
            setPathState(2);
          }
        }
        break;
      case 4:
        // finished
        break;
      default:
        break;
    }
  }

  public void setPathState(int pState) {
    pathState = pState;
    if (pathTimer != null) pathTimer.resetTimer();
  }

  @Override
  public void loop() {
    // loop follower and update states
    if (follower != null) follower.update();
    autonomousPathUpdate();

    // Update battery voltage periodically
    if (voltageTimer != null && voltageTimer.seconds() >= 1.0) {
      batteryVoltage = getBatteryVoltage();
      voltageTimer.reset();
    }

    // Closed-loop launcher velocity control
    if (launcher != null && launchController != null) {
      actualVelocity = launcher.getVelocity();
      if (!(LAUNCHER_DESIRED_VELOCITY == 0 && actualVelocity < 100)) {
        double power =
            (MathUtils.clamp(
                        (launchController.calculate(actualVelocity, LAUNCHER_DESIRED_VELOCITY)
                            + Constants.Launcher.Ks),
                        -1,
                        1)
                    * Constants.Launcher.NOMINAL_BATTERY_VOLTAGE)
                / Math.max(1e-6, batteryVoltage);
        launcher.setPower(power);
      } else {
        launcher.setPower(0);
      }
    }

    // Telemetry
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

  @Override
  public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    feederTimer = new ElapsedTime();
    voltageTimer = new ElapsedTime();

    follower = PedroConstants.createFollower(hardwareMap);
    buildPaths();
    follower.setStartingPose(startPose);

    try {
      launcher = hardwareMap.get(DcMotorEx.class, "launcher");
      leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
      rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

      launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
      rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
      leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

      launchController = new PIDFController(Constants.Launcher.Kp, Constants.Launcher.Ki, Constants.Launcher.Kd, 0);
      voltageSensors = hardwareMap.getAll(VoltageSensor.class);
      batteryVoltage = getBatteryVoltage();
    } catch (Exception e) {
      telemetry.addData("Launcher init error", e.getMessage());
    }

    shotsFired = 0;
    pathState = -1;
  }

  @Override
  public void init_loop() {}

  @Override
  public void start() {
    opmodeTimer.resetTimer();
    setPathState(0);
  }

  @Override
  public void stop() {
    if (launcher != null) launcher.setPower(0);
    if (leftFeeder != null) leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
    if (rightFeeder != null) rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
  }

  private double getBatteryVoltage() {
    if (voltageSensors == null || voltageSensors.isEmpty()) return Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
    double minV = Double.POSITIVE_INFINITY;
    for (VoltageSensor vs : voltageSensors) {
      double v = vs.getVoltage();
      if (v > 0) minV = Math.min(minV, v);
    }
    if (minV == Double.POSITIVE_INFINITY) return Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;
    return minV;
  }
}
