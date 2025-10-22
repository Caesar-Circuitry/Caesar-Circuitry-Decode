package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Config.Utils.LimelightHelper;
import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class drive extends WSubsystem {
  private final Follower follower;
  private final LimelightHelper limelightHelper;
  private final DcMotor FL, FR, BL, BR;

  private final FilteredPIDFController xController;
  private final PIDFController yController;
  private final PIDFController headingController;

  private Pose targetPose;
  private DriveMode currentMode = DriveMode.TELEOP;
  private boolean isHoldingPose = false;

  private double manualX = 0;
  private double manualY = 0;
  private double manualHeading = 0;
  private double flPower, frPower, blPower, brPower;

  private static final double THRESHOLD = 0.1;

  public enum DriveMode {
    TELEOP,
    AUTO
  }

  public drive(Follower follower, HardwareMap hardwareMap) {
    this.follower = follower;
    this.limelightHelper = new LimelightHelper(hardwareMap);

    this.FL = hardwareMap.get(DcMotor.class, PedroConstants.driveConstants.leftFrontMotorName);
    this.FR = hardwareMap.get(DcMotor.class, PedroConstants.driveConstants.rightFrontMotorName);
    this.BL = hardwareMap.get(DcMotor.class, PedroConstants.driveConstants.leftRearMotorName);
    this.BR = hardwareMap.get(DcMotor.class, PedroConstants.driveConstants.rightRearMotorName);

    this.xController =
        new FilteredPIDFController(PedroConstants.followerConstants.coefficientsDrivePIDF);
    this.yController =
        new PIDFController(PedroConstants.followerConstants.coefficientsTranslationalPIDF);
    this.headingController =
        new PIDFController(PedroConstants.followerConstants.coefficientsHeadingPIDF);

    this.targetPose = follower.getPose();
    limelightHelper.enableMT2();
  }

  // === Mode Management ===

  public void setMode(DriveMode mode) {
    if (currentMode != mode) {
      currentMode = mode;
      if (mode == DriveMode.TELEOP) {
        follower.breakFollowing();
        targetPose = follower.getPose();
      }
    }
  }

  public DriveMode getMode() {
    return currentMode;
  }

  // === Teleop Control ===

  public void setManualInput(double x, double y, double heading) {
    this.manualX = x;
    this.manualY = y;
    this.manualHeading = heading;
  }

  public void holdPose(Pose pose) {
    targetPose = pose;
    isHoldingPose = true;
    resetControllers();
  }

  public void enablePoseHolding(boolean enable) {
    isHoldingPose = enable;
    if (enable) {
      targetPose = follower.getPose();
      resetControllers();
    }
  }

  // === Autonomous Control ===

  public void followPath(Path path) {
    setMode(DriveMode.AUTO);
    follower.followPath(path);
  }

  public void followPath(PathChain pathChain) {
    setMode(DriveMode.AUTO);
    follower.followPath(pathChain);
  }

  public boolean isBusy() {
    return follower.isBusy();
  }

  // === Relocalization ===

  public boolean relocalize() {
    if (!limelightHelper.hasMT2Data()) {
      return false;
    }

    Pose3D botPose3D = limelightHelper.getMT2BotPose();
    if (botPose3D == null) {
      return false;
    }

    double x = botPose3D.getPosition().x;
    double y = botPose3D.getPosition().y;
    double heading = Math.toRadians(botPose3D.getOrientation().getYaw());

    follower.setPose(new Pose(x, y, heading));
    targetPose = follower.getPose();
    return true;
  }

  // === Pose Access ===

  public Pose getPose() {
    return follower.getPose();
  }

  public void setPose(Pose pose) {
    follower.setPose(pose);
    targetPose = pose;
  }

  public Follower getFollower() {
    return follower;
  }

  public LimelightHelper getLimelightHelper() {
    return limelightHelper;
  }

  // === Read-Loop-Write ===

  @Override
  public void read() {
    follower.update();
  }

  @Override
  public void loop() {
    Pose currentPose = follower.getPose();
    limelightHelper.updateIMUData(currentPose.getHeading());

    if (currentMode == DriveMode.AUTO) {
      handleAutoMode(currentPose);
    } else {
      handleTeleopMode(currentPose);
    }
  }

  @Override
  public void write() {
    if (currentMode == DriveMode.TELEOP) {
      FL.setPower(flPower);
      FR.setPower(frPower);
      BL.setPower(blPower);
      BR.setPower(brPower);
    }
  }

  // === Internal Methods ===

  private void handleAutoMode(Pose currentPose) {
    if (!follower.isBusy()) {
      targetPose = currentPose;
    }
  }

  private void handleTeleopMode(Pose currentPose) {
    double manualMagnitude = Math.hypot(manualX, manualY) + Math.abs(manualHeading);

    if (manualMagnitude > THRESHOLD) {
      handleManualDrive(currentPose);
    } else if (isHoldingPose) {
      handlePoseHolding(currentPose);
    } else {
      setMotorPowers(0, 0, 0);
    }
  }

  private void handleManualDrive(Pose currentPose) {
    isHoldingPose = false;

    double heading = currentPose.getHeading();
    double rotX = manualX * Math.cos(-heading) - manualY * Math.sin(-heading);
    double rotY = manualX * Math.sin(-heading) + manualY * Math.cos(-heading);

    setMotorPowers(rotX, rotY, manualHeading);
    targetPose = currentPose;
    resetControllers();
  }

  private void handlePoseHolding(Pose currentPose) {
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double headingError = targetPose.getHeading() - currentPose.getHeading();

    xController.updateError(xError);
    yController.updateError(yError);
    headingController.updateError(headingError);

    double xCorrection = xController.run();
    double yCorrection = yController.run();
    double headingCorrection = headingController.run();

    double heading = currentPose.getHeading();
    double rotX = xCorrection * Math.cos(-heading) - yCorrection * Math.sin(-heading);
    double rotY = xCorrection * Math.sin(-heading) + yCorrection * Math.cos(-heading);

    setMotorPowers(rotX, rotY, headingCorrection);
  }

  private void setMotorPowers(double x, double y, double heading) {
    flPower = y + x + heading;
    frPower = y - x - heading;
    blPower = y - x + heading;
    brPower = y + x - heading;

    double maxPower =
        Math.max(
            Math.abs(flPower),
            Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));

    if (maxPower > 1.0) {
      flPower /= maxPower;
      frPower /= maxPower;
      blPower /= maxPower;
      brPower /= maxPower;
    }
  }

  private void resetControllers() {
    xController.reset();
    yController.reset();
    headingController.reset();
  }

  public void stop() {
    follower.breakFollowing();
    setMotorPowers(0, 0, 0);
    currentMode = DriveMode.TELEOP;
    isHoldingPose = false;
  }
}
