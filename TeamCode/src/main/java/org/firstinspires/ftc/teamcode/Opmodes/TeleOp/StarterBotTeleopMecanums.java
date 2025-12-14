package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.Constants;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.MathUtils;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {
  private PIDFController launchController;
  private double actualVelocity = 0;
  private boolean brakeFlag = false;

  List<LynxModule> allHubs;
  private List<VoltageSensor> voltageSensors;
  private double batteryVoltage = Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;

  private double LAUNCHER_DESIRED_VELOCITY = 0;

  private DcMotor leftFrontDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightBackDrive = null;
  private DcMotorEx launcher = null;
  private CRServo leftFeeder = null;
  private CRServo rightFeeder = null;

  ElapsedTime feederTimer = new ElapsedTime();
  ElapsedTime voltageTimer = new ElapsedTime();

  private enum LaunchState {
    IDLE,
    SPIN_UP,
    LAUNCH,
    LAUNCHING,
  }

  private LaunchState launchState;
  private boolean prevRightBumper = false;

  double leftFrontPower;
  double rightFrontPower;
  double leftBackPower;
  double rightBackPower;

  // New flag to ensure we rumble only once when the launcher reaches speed
  private boolean launcherReadyNotified = false;

  @Override
  public void init() {
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    launchState = LaunchState.IDLE;
    voltageSensors = hardwareMap.getAll(VoltageSensor.class);

    leftFrontDrive = hardwareMap.get(DcMotor.class, "FLM");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "FRM");
    leftBackDrive = hardwareMap.get(DcMotor.class, "BLM");
    rightBackDrive = hardwareMap.get(DcMotor.class, "BRM");
    launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
    rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
    launchController =
        new PIDFController(Constants.Launcher.Kp, Constants.Launcher.Ki, Constants.Launcher.Kd, 0);

    leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    leftFrontDrive.setZeroPowerBehavior(BRAKE);
    rightFrontDrive.setZeroPowerBehavior(BRAKE);
    leftBackDrive.setZeroPowerBehavior(BRAKE);
    rightBackDrive.setZeroPowerBehavior(BRAKE);
    launcher.setZeroPowerBehavior(BRAKE);

    leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
    rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);

    leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    // ensure notification flag is cleared at init
    launcherReadyNotified = false;

    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void start() {
    voltageTimer.reset();
  }

  @Override
  public void loop() {

    if (voltageTimer.seconds() >= 5.0) {
      batteryVoltage = getBatteryVoltage();
      voltageTimer.reset();
    }

    actualVelocity = launcher.getVelocity();
    if (!(LAUNCHER_DESIRED_VELOCITY == 0 && !brakeFlag)) {
      launcher.setPower(
          (MathUtils.clamp(
                      (launchController.calculate(actualVelocity, LAUNCHER_DESIRED_VELOCITY)
                          + Constants.Launcher.Ks),
                      -1,
                      1)
                  * Constants.Launcher.NOMINAL_BATTERY_VOLTAGE)
              / batteryVoltage);
      brakeFlag = false;
    } else {
      launcher.setPower(0);
    }

    // LED and rumble logic:
    // - Show green and rumble once when the launcher is spinning and desired velocity is nonzero.
    // - Keep LED green while launcher still above MIN_VELOCITY.
    // - Reset notification and show red when launcher is off.
    if (LAUNCHER_DESIRED_VELOCITY > 0) {
      if (actualVelocity > Constants.Launcher.MIN_VELOCITY) {
        if (!launcherReadyNotified) {
          gamepad1.rumbleBlips(3);
          launcherReadyNotified = true;
        }
        gamepad1.setLedColor(0, 255, 0, 1000);
      } else {
        gamepad1.setLedColor(255, 0, 0, 1000);
      }
    } else {
      // launcher not requested: reset notification and show red
      launcherReadyNotified = false;
      gamepad1.setLedColor(255, 0, 0, 1000);
    }

    // drive
    mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

    // launcher speed control
    if (gamepad1.y) {
      LAUNCHER_DESIRED_VELOCITY = Constants.Launcher.TARGET_VELOCITY;
    } else if (gamepad1.square) { // stop flywheel
      LAUNCHER_DESIRED_VELOCITY = 0;
    }
    if (gamepad1.left_bumper) {
      LAUNCHER_DESIRED_VELOCITY = 0;
      brakeFlag = true;
    }

    boolean rightBumperPressed = gamepad1.right_bumper && !prevRightBumper;
    launch(rightBumperPressed);
    prevRightBumper = gamepad1.right_bumper;

    telemetry.addData("State", launchState);
    telemetry.addData("motorSpeed", actualVelocity);
    telemetry.addData("launcherPower", launcher.getPower());
    for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
    }
  }

  void mecanumDrive(double forward, double strafe, double rotate) {
    double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

    leftFrontPower = (forward + strafe + rotate) / denominator;
    rightFrontPower = (forward - strafe - rotate) / denominator;
    leftBackPower = (forward - strafe + rotate) / denominator;
    rightBackPower = (forward + strafe - rotate) / denominator;

    leftFrontDrive.setPower(leftFrontPower);
    rightFrontDrive.setPower(rightFrontPower);
    leftBackDrive.setPower(leftBackPower);
    rightBackDrive.setPower(rightBackPower);
  }

  void launch(boolean shotRequested) {
    switch (launchState) {
      case IDLE:
        if (shotRequested) {
          launchState = LaunchState.SPIN_UP;
        }
        leftFeeder.setPower(Constants.Launcher.FEEDER_REVERSE);
        rightFeeder.setPower(Constants.Launcher.FEEDER_REVERSE);
        break;
      case SPIN_UP:
        LAUNCHER_DESIRED_VELOCITY = Constants.Launcher.TARGET_VELOCITY;
        if (actualVelocity > Constants.Launcher.MIN_VELOCITY) {
          launchState = LaunchState.LAUNCH;
        }
        leftFeeder.setPower(Constants.Launcher.FEEDER_REVERSE);
        rightFeeder.setPower(Constants.Launcher.FEEDER_REVERSE);
        break;
      case LAUNCH:
        leftFeeder.setPower(Constants.Launcher.FEEDER_POWER);
        rightFeeder.setPower(Constants.Launcher.FEEDER_POWER);
        feederTimer.reset();
        launchState = LaunchState.LAUNCHING;
        break;
      case LAUNCHING:
        if (feederTimer.seconds() > Constants.Launcher.FEED_TIME_SECONDS) {
          launchState = LaunchState.IDLE;
          leftFeeder.setPower(Constants.Launcher.FEEDER_REVERSE);
          rightFeeder.setPower(Constants.Launcher.FEEDER_REVERSE);
        }
        break;
    }
  }

  private double getBatteryVoltage() {
    if (voltageSensors == null || voltageSensors.isEmpty()) return 0.0;
    double minV = Double.POSITIVE_INFINITY;
    for (VoltageSensor vs : voltageSensors) {
      double v = vs.getVoltage();
      if (v > 0) minV = Math.min(minV, v);
    }
    if (minV == Double.POSITIVE_INFINITY) {
      return 0.0;
    }
    return minV;
  }
}
