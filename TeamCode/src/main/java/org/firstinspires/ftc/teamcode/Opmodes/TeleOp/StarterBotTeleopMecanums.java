/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Opmodes.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

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

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
// @Disabled
public class StarterBotTeleopMecanums extends OpMode {
  // shared launcher constants moved to Constants.Launcher
  private PIDFController launchController;
  // use Constants.Launcher.Kp/Ki/Kd/Ks when creating controller
  private double actualVelocity = 0;

  List<LynxModule> allHubs;
  private List<VoltageSensor> voltageSensors;
  // use Constants.Launcher.NOMINAL_BATTERY_VOLTAGE for nominal voltage
  private double batteryVoltage = Constants.Launcher.NOMINAL_BATTERY_VOLTAGE;

  private double LAUNCHER_DESIRED_VELOCITY = 0;

  // Declare OpMode members.
  private DcMotor leftFrontDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightBackDrive = null;
  private DcMotorEx launcher = null;
  private CRServo leftFeeder = null;
  private CRServo rightFeeder = null;

  ElapsedTime feederTimer = new ElapsedTime();
  ElapsedTime voltageTimer = new ElapsedTime();

  /*
   * TECH TIP: State Machines
   * We use a "state machine" to control our launcher motor and feeder servos in this program.
   * The first step of a state machine is creating an enum that captures the different "states"
   * that our code can be in.
   * The core advantage of a state machine is that it allows us to continue to loop through all
   * of our code while only running specific code when it's necessary. We can continuously check
   * what "State" our machine is in, run the associated code, and when we are done with that step
   * move on to the next state.
   * This enum is called the "LaunchState". It reflects the current condition of the shooter
   * motor and we move through the enum when the user asks our code to fire a shot.
   * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
   * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
   * We can use higher level code to cycle through these states. But this allows us to write
   * functions and autonomous routines in a way that avoids loops within loops, and "waits".
   */
  private enum LaunchState {
    IDLE,
    SPIN_UP,
    LAUNCH,
    LAUNCHING,
  }

  private LaunchState launchState;
  // simple edge detector for right bumper (one-shot press)
  private boolean prevRightBumper = false;

  // Setup a variable for each drive wheel to save power level for telemetry
  double leftFrontPower;
  double rightFrontPower;
  double leftBackPower;
  double rightBackPower;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    launchState = LaunchState.IDLE;
    voltageSensors = hardwareMap.getAll(VoltageSensor.class);

    /*
     * Initialize the hardware variables. Note that the strings used here as parameters
     * to 'get' must correspond to the names assigned during the robot configuration
     * step.
     */
    leftFrontDrive = hardwareMap.get(DcMotor.class, "FLM");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "FRM");
    leftBackDrive = hardwareMap.get(DcMotor.class, "BLM");
    rightBackDrive = hardwareMap.get(DcMotor.class, "BRM");
    launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
    rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
    launchController =
        new PIDFController(Constants.Launcher.Kp, Constants.Launcher.Ki, Constants.Launcher.Kd, 0);

    /*
     * To drive forward, most robots need the motor on one side to be reversed,
     * because the axles point in opposite directions. Pushing the left stick forward
     * MUST make robot go forward. So adjust these two lines based on your first test drive.
     * Note: The settings here assume direct drive on left and right wheels. Gear
     * Reduction or 90 Deg drives may require direction flips
     */
    leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    /*
     * Here we set our launcher to the RUN_USING_ENCODER runmode.
     * If you notice that you have no control over the velocity of the motor, it just jumps
     * right to a number much higher than your set point, make sure that your encoders are plugged
     * into the port right beside the motor itself. And that the motors polarity is consistent
     * through any wiring.
     */
    // Reset encoder then use RUN_WITHOUT_ENCODER so getVelocity() reports correctly
    launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    /*
     * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
     * slow down much faster when it is coasting. This creates a much more controllable
     * drivetrain. As the robot stops much quicker.
     */
    leftFrontDrive.setZeroPowerBehavior(BRAKE);
    rightFrontDrive.setZeroPowerBehavior(BRAKE);
    leftBackDrive.setZeroPowerBehavior(BRAKE);
    rightBackDrive.setZeroPowerBehavior(BRAKE);
    launcher.setZeroPowerBehavior(FLOAT);

    /*
     * set Feeders to an initial value to initialize the servo controller
     */
    leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
    rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);

    /*
     * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
     * both work to feed the ball into the robot.
     */
    leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

    /*
     * Tell the driver that initialization is complete.
     */
    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void start() {
    voltageTimer.reset();
  }

  /*
   * Code to run REPEATEDLY after the driver hits START but before they hit STOP
   */
  @Override
  public void loop() {

    if (voltageTimer.seconds() >= 5.0) {
      batteryVoltage = getBatteryVoltage();
      voltageTimer.reset();
    }

    actualVelocity = launcher.getVelocity();
    if (!(LAUNCHER_DESIRED_VELOCITY == 0)) {
      launcher.setPower(
          (MathUtils.clamp(
                      (launchController.calculate(actualVelocity, LAUNCHER_DESIRED_VELOCITY)
                          + Constants.Launcher.Ks),
                      -1,
                      1)
                  * Constants.Launcher.NOMINAL_BATTERY_VOLTAGE)
              / batteryVoltage);
    } else {
      launcher.setPower(0);
    }

    if (actualVelocity > Constants.Launcher.MIN_VELOCITY && launchState == LaunchState.SPIN_UP) {
      gamepad1.rumbleBlips(3);
      gamepad1.setLedColor(0, 255, 0, 1000);
    } else {
      gamepad1.setLedColor(255, 0, 0, 1000);
    }

    // drive
    mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

    // launcher speed control
    if (gamepad1.y) {
      LAUNCHER_DESIRED_VELOCITY = Constants.Launcher.TARGET_VELOCITY;
    } else if (gamepad1.b) { // stop flywheel
      LAUNCHER_DESIRED_VELOCITY = 0;
    }

    // Detect a one-shot press of the right bumper and call launch with that event
    boolean rightBumperPressed = gamepad1.right_bumper && !prevRightBumper;
    launch(rightBumperPressed);
    prevRightBumper = gamepad1.right_bumper;

    // telemetry and housekeeping
    telemetry.addData("State", launchState);
    telemetry.addData("motorSpeed", actualVelocity);
    telemetry.addData("launcherPower", launcher.getPower());
    for (LynxModule hub : allHubs) {
      hub.clearBulkCache();
    }
  }

  void mecanumDrive(double forward, double strafe, double rotate) {

    /* the denominator is the largest motor power (absolute value) or 1
     * This ensures all the powers maintain the same ratio,
     * but only if at least one is out of the range [-1, 1]
     */
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
        break;
      case SPIN_UP:
        LAUNCHER_DESIRED_VELOCITY = Constants.Launcher.TARGET_VELOCITY;
        if (actualVelocity > Constants.Launcher.MIN_VELOCITY) {
          launchState = LaunchState.LAUNCH;
        }
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
          leftFeeder.setPower(Constants.Launcher.FEEDER_STOP);
          rightFeeder.setPower(Constants.Launcher.FEEDER_STOP);
        }
        break;
    }
  }

  private double getBatteryVoltage() {
    if (voltageSensors == null || voltageSensors.isEmpty()) return 0.0;
    double minV = Double.POSITIVE_INFINITY;
    for (VoltageSensor vs : voltageSensors) {
      double v = vs.getVoltage();
      // some sensors may return 0 if unknown; skip zeros unless all zeros
      if (v > 0) minV = Math.min(minV, v);
    }
    if (minV == Double.POSITIVE_INFINITY) {
      // all sensors returned 0 or list empty
      return 0.0;
    }
    return minV;
  }
}
