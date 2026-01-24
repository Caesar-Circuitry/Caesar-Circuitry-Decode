package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.*;
import static org.firstinspires.ftc.teamcode.Config.Utils.TurretMath.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Config.Utils.AxonEncoder;
import org.firstinspires.ftc.teamcode.Config.Utils.AnglePIDF;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import java.util.LinkedList;


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
  private LinkedList<TelemetryPacket> telemetryPackets;
  private double unwrappedServoAngle = 0;
  private double desiredTurretAngle = 0;
  private double safeTurretAngle = 0;



  public Turret(HardwareMap hardwareMap, Follower follower) {
      servo = hardwareMap.get(CRServo.class, servoName);
      servo2 = hardwareMap.get(CRServo.class, servoName2);
      turretEncoder = new AxonEncoder(hardwareMap.get(com.qualcomm.robotcore.hardware.AnalogInput.class, servoEncoderName), gearRatio, 180.0);
      this.follower = follower;
      angleController = new AnglePIDF(kP, kI, kD, kF_left, kF_right);
      telemetryPackets = new LinkedList<TelemetryPacket>();
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

    // Log telemetry if enabled
    if (logTelemetry) {
        telemetryPackets.clear(); // Clear previous packets

        // Configuration
        telemetryPackets.addLast(new TelemetryPacket("Target Angle (Field)", targetAngle));
        telemetryPackets.addLast(new TelemetryPacket("Target Angle (Robot)", robotRelativeTargetAngle));
        telemetryPackets.addLast(new TelemetryPacket("Pinpoint Tracking", trackPinpoint));

        // Sensor readings
        telemetryPackets.addLast(new TelemetryPacket("Heading", heading));
        telemetryPackets.addLast(new TelemetryPacket("Raw Encoder", turretEncoder.getRawAngle()));

        // Loop calculations
        telemetryPackets.addLast(new TelemetryPacket("Unwrapped Servo", unwrappedServoAngle));
        telemetryPackets.addLast(new TelemetryPacket("Current Turret", currentTurretAngle));
        telemetryPackets.addLast(new TelemetryPacket("Desired Turret", desiredTurretAngle));
        telemetryPackets.addLast(new TelemetryPacket("Safe Turret", safeTurretAngle));
        telemetryPackets.addLast(new TelemetryPacket("Target Servo", targetServoAngle));

        // Control output
        telemetryPackets.addLast(new TelemetryPacket("Servo Error", servoError));
        telemetryPackets.addLast(new TelemetryPacket("Servo Power", servoPower));

        // PIDF state
        telemetryPackets.addLast(new TelemetryPacket("PIDF Error", angleController.getLastError()));
        telemetryPackets.addLast(new TelemetryPacket("PIDF Integral", angleController.getIntegral()));
    }
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

  @Override
  public LinkedList<TelemetryPacket> getTelemetry(){
      return telemetryPackets;
  }

  public InstantCommand TargetAngle(double Angle){
      return new InstantCommand(()->setTargetAngle(Angle));
  }

  /**
   * Minimal getter required by Vision subsystem to compute camera orientation.
   * @return current turret angle in degrees (robot-relative)
   */
  public double getCurrentTurretAngle() {
      return currentTurretAngle;
  }
}
