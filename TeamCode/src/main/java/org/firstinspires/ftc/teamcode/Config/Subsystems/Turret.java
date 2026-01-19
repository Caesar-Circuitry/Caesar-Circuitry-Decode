package org.firstinspires.ftc.teamcode.Config.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.Constants.Turret.*;
import static org.firstinspires.ftc.teamcode.Config.Utils.TurretMath.*;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;


public class Turret extends WSubsystem {
    private CRServo servo;
    private CRServo servo2;
    private AnalogInput servoEncoder;
    private PIDFController angleController;
    private Follower follower;

    // Configurable parameters
    private double targetAngle = 0; // field-relative angle the turret should face

    // Read variables (sensor values)
    private double heading = 0;
    private double currentServoAngle = 0;

    private double desiredTurretAngle;
    private double desiredServoAngle;
    private double currentTurretAngle;
    private double wrappedServoError;
    private double adjustedTarget;

    // Write variables (actuator commands)
    private double servoPower = 0;



  public Turret(HardwareMap hardwareMap, Follower follower) {
      servo = hardwareMap.get(CRServo.class, servoName);
      servo2 = hardwareMap.get(CRServo.class, servoName2);
      servoEncoder = hardwareMap.get(AnalogInput.class, servoEncoderName);
      this.follower = follower;
      angleController = new PIDFController(kP, kI, kD, 0);
  }

  @Override
  public void read() {
    // Read robot heading from follower
    heading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

    // Read current servo angle from absolute encoder (in degrees)
    currentServoAngle = getUnwrappedServoAngle(servoEncoder);
  }

  @Override
  public void loop() {

    // Calculate desired turret angle (robot-relative) to point to targetAngle (field-relative)
    double desiredTurretAngleRaw = targetAngle - heading;
    double wrappedDesiredTurretAngle = wrap180(desiredTurretAngleRaw);

    // Find the closest valid turret angle within the 270-degree range (-135 to +135)
    desiredTurretAngle = getClosestAngleInRange(wrappedDesiredTurretAngle, -135.0, 135.0);

    // Convert desired turret angle to desired servo angle using gear ratio
    desiredServoAngle = desiredTurretAngle * gearRatio;

    // Convert current servo angle to equivalent turret angle for telemetry
    currentTurretAngle = currentServoAngle / gearRatio;

    // Calculate servo power using PID control
    double servoError = desiredServoAngle - currentServoAngle;
    wrappedServoError = wrap180(servoError);

    // Calculate the target position that's closest to current via wrapping
    adjustedTarget = currentServoAngle + wrappedServoError;

    // Use PID controller with current position and adjusted target
    angleController.setSetPoint(adjustedTarget);
    servoPower = angleController.calculate(currentServoAngle);

    // Add direction-specific feedforward term to overcome friction/deadband
    // Both directions add positive feedforward in the direction of motion
    if (wrappedServoError > 0) {
      servoPower += kF_left;  // Moving left (positive power)
    } else if (wrappedServoError < 0) {
      servoPower += kF_right; // Moving right (negative power)
    }

    // Clamp servo power to valid range for CRServo
    servoPower = clamp(servoPower, -1.0, 1.0);
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
  }

  /**
   * Get the current target angle (field-relative)
   * @return Target angle in degrees
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Get the current turret angle (robot-relative)
   * @return Current turret angle in degrees
   */
  public double getCurrentTurretAngle() {
    return currentTurretAngle;
  }

  /**
   * Get the desired turret angle after range limiting
   * @return Desired turret angle in degrees
   */
  public double getDesiredTurretAngle() {
    return desiredTurretAngle;
  }


  /**
   * Get the current servo power being applied
   * @return Servo power from -1.0 to 1.0
   */
  public double getServoPower() {
    return servoPower;
  }

  /**
   * Get the error between desired and current servo angle (wrapped)
   * @return Error in degrees
   */
  public double getServoError() {
    return wrappedServoError;
  }

  public InstantCommand TargetAngle(double Angle){
      return new InstantCommand(()->setTargetAngle(Angle));
  }
}
