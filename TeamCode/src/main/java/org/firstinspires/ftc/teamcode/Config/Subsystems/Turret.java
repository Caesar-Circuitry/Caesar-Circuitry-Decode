package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.*;
import static org.firstinspires.ftc.teamcode.Config.Utils.TurretMath.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.Config.Utils.AxonEncoder;
import org.firstinspires.ftc.teamcode.Config.Utils.AnglePIDF;


public class Turret extends WSubsystem {
    private CRServo servo;
    private CRServo servo2;
    private AxonEncoder turretEncoder;
    private AnglePIDF angleController;
    private Follower follower;

  // Configuration
  private double targetAngle = 0;
  private double robotRelativeTargetAngle = 0;
  private boolean trackPinpoint = false;

  // State
  private double heading = 0;
  private double currentTurretAngle = 0;
  private double targetServoAngle = 0;
  private double servoError = 0;
  private double servoPower = 0;

  // Loop intermediate values (for telemetry/debugging)
  private double unwrappedServoAngle = 0;
  private double desiredTurretAngle = 0;
  private double safeTurretAngle = 0;



  public Turret(HardwareMap hardwareMap, Follower follower) {
      servo = hardwareMap.get(CRServo.class, servoName);
      servo2 = hardwareMap.get(CRServo.class, servoName2);
      turretEncoder = new AxonEncoder(hardwareMap.get(com.qualcomm.robotcore.hardware.AnalogInput.class, servoEncoderName), gearRatio, 180.0);
      this.follower = follower;
      angleController = new AnglePIDF(kP, kI, kD, kF_left, kF_right);

      targetServoAngle = turretEncoder.getUnwrappedEncoderAngle();
  }

  @Override
  public void read() {
    // Read robot heading from follower
    heading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

    // Update encoder - handles wrap detection and accumulation internally
    turretEncoder.update();
  }

  @Override
  public void loop() {
    unwrappedServoAngle = turretEncoder.getUnwrappedEncoderAngle();
    currentTurretAngle = unwrappedServoAngle / gearRatio;

    // Determine desired turret angle based on mode
    desiredTurretAngle = trackPinpoint
      ? wrap180(targetAngle - heading)
      : robotRelativeTargetAngle;

    // Apply safety limits and find target
    safeTurretAngle = getSafeTurretAngle(desiredTurretAngle, wrap180(currentTurretAngle));
    targetServoAngle = getClosestServoTarget(unwrappedServoAngle, safeTurretAngle * gearRatio);

    // Calculate and apply PIDF control
    servoError = targetServoAngle - unwrappedServoAngle;
    angleController.setSetPoint(targetServoAngle);
    servoPower = clamp(angleController.calculate(unwrappedServoAngle), -1.0, 1.0);
  }


  @Override
  public void write() {
    // Set power to both servos
    servo.setPower(servoPower);
    servo2.setPower(servoPower);
  }

  // Public methods for controlling and querying turret state

  /**
   * Set the target angle (field-relative) for the turret to point to
   * @param angle Target angle in degrees (field-relative)
   */
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
    // Calculate and store the robot-relative equivalent for use in manual mode
    this.robotRelativeTargetAngle = wrap180(angle - heading);
    this.trackPinpoint = false; // Switch to manual mode when target is set
  }

  /**
   * Enable tracking with pinpoint odometry
   * Turret will continuously update its target based on the robot's heading
   */
  public void enablePinpointTracking() {
    this.trackPinpoint = true;
  }

  /**
   * Disable tracking with pinpoint odometry
   * Turret will only move when setTargetAngle() is called
   */
  public void disablePinpointTracking() {
    this.trackPinpoint = false;
  }

  /**
   * Check if pinpoint tracking is enabled
   * @return true if turret is tracking with pinpoint, false if in manual mode
   */
  public boolean isPinpointTrackingEnabled() {
    return trackPinpoint;
  }

  // Getters for debugging and telemetry

  /**
   * Get the target angle (field-relative)
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Get the robot-relative target angle
   * @return Robot-relative target angle in degrees
   */
  public double getRobotRelativeTargetAngle() {
    return robotRelativeTargetAngle;
  }

  /**
   * Get the robot heading from pinpoint
   * @return Robot heading in degrees
   */
  public double getHeading() {
    return heading;
  }

  /**
   * Get the current turret angle (robot-relative)
   * @return Current turret angle in degrees
   */
  public double getCurrentTurretAngle() {
    return currentTurretAngle;
  }

  /**
   * Get the current unwrapped turret angle
   * @return Current unwrapped turret angle in degrees
   */
  public double getUnwrappedTurretAngle() {
    return turretEncoder.getUnwrappedOutputAngle();
  }

  /**
   * Get the target servo angle (unwrapped)
   * @return Target servo angle in degrees
   */
  public double getTargetServoAngle() {
    return targetServoAngle;
  }

  /**
   * Get the current servo angle (unwrapped)
   * @return Current servo angle in degrees
   */
  public double getCurrentServoAngle() {
    return turretEncoder.getUnwrappedEncoderAngle();
  }

  /**
   * Get the error between desired and current servo angle (unwrapped)
   * @return Error in degrees
   */
  public double getServoError() {
    return servoError;
  }

  /**
   * Get the current servo power being applied
   * @return Servo power from -1.0 to 1.0
   */
  public double getServoPower() {
    return servoPower;
  }

  /**
   * Get the raw encoder angle (0-360)
   * @return Raw encoder angle in degrees
   */
  public double getRawEncoderAngle() {
    return turretEncoder.getRawAngle();
  }

  /**
   * Get the wrapped turret angle (-180 to 180)
   * @return Wrapped turret angle in degrees
   */
  public double getWrappedTurretAngle() {
    return wrap180(currentTurretAngle);
  }

  /**
   * Get the unwrapped servo angle (intermediate calculation)
   * @return Unwrapped servo angle in degrees
   */
  public double getUnwrappedServoAngle() {
    return unwrappedServoAngle;
  }

  /**
   * Get the desired turret angle (before safety limits)
   * @return Desired turret angle in degrees
   */
  public double getDesiredTurretAngle() {
    return desiredTurretAngle;
  }

  /**
   * Get the safe turret angle (after safety limits applied)
   * @return Safe turret angle in degrees
   */
  public double getCalculatedSafeTurretAngle() {
    return safeTurretAngle;
  }

  public InstantCommand TargetAngle(double Angle){
      return new InstantCommand(()->setTargetAngle(Angle));
  }
}
