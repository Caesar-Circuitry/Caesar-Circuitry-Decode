package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Constants;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class Launcher extends WSubsystem {

  private MotorGroup Flywheel;
  private MotorEx FlywheelLead;
  private MotorEx FlywheelFollow;

  private double flywheelVelocity = 0.0;
  private double flywheelTargetVelocity = 0.0; // Ticks per second max is 2800 for bare motor
  private double speed; // keep this fully hidden used for calculations only

  public Launcher(HardwareMap hardwareMap) {
    FlywheelLead =
        new MotorEx(hardwareMap, Constants.Launcher.FLYWHEEL_MOTOR_LEAD, Motor.GoBILDA.BARE);
    FlywheelFollow =
        new MotorEx(hardwareMap, Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW, Motor.GoBILDA.BARE);

    FlywheelLead.setInverted(Constants.Launcher.FLYWHEEL_MOTOR_LEAD_INVERTED);
    FlywheelFollow.setInverted(Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW_INVERTED);

    Flywheel = new MotorGroup(FlywheelLead, FlywheelFollow);
    Flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    Flywheel.stopAndResetEncoder();
    Flywheel.setRunMode(Motor.RunMode.VelocityControl);

    Flywheel.setVeloCoefficients(
        Constants.Launcher.kP, Constants.Launcher.kI, Constants.Launcher.kD);
    Flywheel.setFeedforwardCoefficients(
        Constants.Launcher.kS, Constants.Launcher.kV, Constants.Launcher.kA);
  }

  @Override
  public void read() {
    this.flywheelVelocity = Flywheel.getVelocity();
  }

  @Override
  public void loop() {
    speed = this.flywheelTargetVelocity / Flywheel.ACHIEVABLE_MAX_TICKS_PER_SECOND;
  }

  @Override
  public void write() {
    Flywheel.set(speed);
  }

  public double getFlywheelTargetVelocity() {
    return flywheelTargetVelocity;
  }

  public void setFlywheelTargetVelocity(double flywheelTargetVelocity) {
    this.flywheelTargetVelocity = flywheelTargetVelocity;
  }

  public double getFlywheelVelocity() {
    return flywheelVelocity;
  }

  public void setFlywheelVelocity(double flywheelVelocity) {
    this.flywheelVelocity = flywheelVelocity;
  }
}
