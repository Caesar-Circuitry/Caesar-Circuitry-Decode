// package org.firstinspires.ftc.teamcode.Config.Subsystems;
//
// import org.firstinspires.ftc.teamcode.Config.Constants;
//
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.seattlesolvers.solverslib.controller.PIDFController;
// import com.seattlesolvers.solverslib.util.MathUtils;
//
// public class Launcher extends WSubsystem {
//
//  private final DcMotorEx launcherMotor;
//  private final PIDFController launcherController;
//  private final CRServo leftFeeder;
//  private final CRServo rightFeeder;
//  private final ElapsedTime feederTimer = new ElapsedTime();
//
//  private double launcherSetVelocity = 0;
//  private double motorPower = 0;
//  private double launcherActualVelocity = 0;
//  private double feederPower = 0;
//
//  private boolean shotRequested = false;
//  private boolean burstMode = false;
//  private int burstCount = 0;
//  private int burstShotsFired = 0;
//
//  private enum LaunchState {
//    IDLE,
//    SPIN_UP,
//    FEEDING,
//    COOLDOWN
//  }
//
//  private LaunchState launchState = LaunchState.IDLE;
//
//  public Launcher(HardwareMap hwMap) {
//    this.launcherMotor = hwMap.get(DcMotorEx.class, "launcherMotor");
//    this.launcherMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    this.launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//
//    this.leftFeeder = hwMap.get(CRServo.class, "leftFeeder");
//    this.rightFeeder = hwMap.get(CRServo.class, "rightFeeder");
//
//    this.launcherController =
//        new PIDFController(
//            Constants.Launcher.kP,
//            Constants.Launcher.kI,
//            Constants.Launcher.kD,
//            Constants.Launcher.kF);
//  }
//
//  @Override
//  public void read() {
//    launcherActualVelocity = launcherMotor.getVelocity();
//  }
//
//  @Override
//  public void loop() {
//    switch (launchState) {
//      case IDLE:
//        launcherSetVelocity = 0;
//        feederPower = 0;
//        if (shotRequested) {
//          launchState = LaunchState.SPIN_UP;
//          shotRequested = false;
//          burstShotsFired = 0;
//        }
//        break;
//
//      case SPIN_UP:
//        launcherSetVelocity = Constants.Launcher.TARGET_VELOCITY;
//        if (launcherActualVelocity >= Constants.Launcher.MIN_VELOCITY) {
//          feederPower = Constants.Launcher.Feeder_FULL_SPEED;
//          feederTimer.reset();
//          launchState = LaunchState.FEEDING;
//        }
//        break;
//
//      case FEEDING:
//        if (feederTimer.seconds() >= Constants.Launcher.FEED_TIME_SECONDS) {
//          feederPower = 0;
//          feederTimer.reset();
//          burstShotsFired++;
//          launchState = LaunchState.COOLDOWN;
//        }
//        break;
//
//      case COOLDOWN:
//        if (feederTimer.seconds() >= Constants.Launcher.COOLDOWN_TIME_SECONDS) {
//          if (burstMode && burstShotsFired < burstCount) {
//            // Continue burst fire - go back to spin up
//            launchState = LaunchState.SPIN_UP;
//          } else {
//            // Burst complete or single shot - return to idle
//            launchState = LaunchState.IDLE;
//            burstMode = false;
//          }
//        }
//        break;
//    }
//
//    motorPower = launcherController.calculate(launcherActualVelocity, launcherSetVelocity);
//    motorPower = MathUtils.clamp(motorPower, -1, 1);
//  }
//
//  @Override
//  public void write() {
//    launcherMotor.setPower(motorPower);
//    leftFeeder.setPower(feederPower);
//    rightFeeder.setPower(feederPower);
//  }
//
//  public void launch() {
//    shotRequested = true;
//    burstMode = false;
//  }
//
//  public void launchBurst(int shotCount) {
//    shotRequested = true;
//    burstMode = true;
//    burstCount = shotCount;
//  }
//
//  public void stop() {
//    shotRequested = false;
//    burstMode = false;
//    launchState = LaunchState.IDLE;
//  }
//
//  public boolean isReadyToLaunch() {
//    return launcherActualVelocity >= Constants.Launcher.MIN_VELOCITY;
//  }
//
//  public boolean isLaunching() {
//    return launchState == LaunchState.FEEDING;
//  }
//
//  public boolean isBurstActive() {
//    return burstMode && launchState != LaunchState.IDLE;
//  }
//
//  public int getBurstShotsFired() {
//    return burstShotsFired;
//  }
//
//  public LaunchState getState() {
//    return launchState;
//  }
//
//  public double getVelocity() {
//    return launcherActualVelocity;
//  }
// }
