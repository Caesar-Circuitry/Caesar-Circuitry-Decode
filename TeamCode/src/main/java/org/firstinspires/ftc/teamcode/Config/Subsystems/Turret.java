package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;

import com.seattlesolvers.solverslib.controller.SquIDFController;


public class Turret extends WSubsystem {
  private Servo turretServoLead;
  private Servo turretServoFollow;

  private Limelight3A limelight3A;

  private AbsoluteAnalogEncoder turretServoLeadEncoder;
  private AbsoluteAnalogEncoder turretServoFollowEncoder;

  private double turretServoLeadPosition = 0.0;
  private double turretServoLeadCurrentPosition = 0.0;
  private double turretServoFollowPosition = 0.0;
  private double turretServoFollowCurrentPosition = 0.0;

  private boolean autoTrackEnabled = false;
  private double Tx = 0;

  private SquIDFController squid;

  public Turret(HardwareMap hardwareMap) {
    turretServoLead = hardwareMap.get(Servo.class, "turretServoLead");
    turretServoFollow = hardwareMap.get(Servo.class, "turretServoFollow");

    turretServoLeadEncoder =
        new AbsoluteAnalogEncoder(hardwareMap, "turretServoLeadEncoder", 270, AngleUnit.DEGREES);
    turretServoFollowEncoder =
        new AbsoluteAnalogEncoder(hardwareMap, "turretServoFollowEncoder", 270, AngleUnit.DEGREES);
    turretServoFollow.setDirection(Servo.Direction.REVERSE);
    turretServoFollowEncoder.setReversed(true);
      limelight3A = hardwareMap.get(Limelight3A.class, "limelight3a");

      squid = new SquIDFController(Constants.Turret.kP,Constants.Turret.kI,Constants.Turret.kD,Constants.Turret.kF);


  }

  @Override
  public void read() {
    turretServoLeadCurrentPosition = turretServoLeadEncoder.getCurrentPosition();
    turretServoFollowCurrentPosition = turretServoFollowEncoder.getCurrentPosition();
  }

  @Override
  public void loop() {

  }

  private double clampAngleDeg(double angleDeg) {
    if (angleDeg > 135.0) return 135.0;
    if (angleDeg < -135.0) return -135.0;
    return angleDeg;
  }

  public void setTurretFollowAngleDeg(double angleDeg) {
    this.turretServoFollowPosition = clampAngleDeg(angleDeg);
  }

  private double angleDegToServoPos(double angleDeg) {
    double pos = (angleDeg + 135.0) / 270.0;
    if (pos < 0.0) return 0.0;
    if (pos > 1.0) return 1.0;
    return pos;
  }

  @Override
  public void write() {
    double leadServoPos = angleDegToServoPos(turretServoLeadPosition);
    double followServoPos = angleDegToServoPos(turretServoFollowPosition);

    turretServoLead.setPosition(leadServoPos);
    turretServoFollow.setPosition(followServoPos);
  }

  public void enableAutoTrack() {
    this.autoTrackEnabled = true;
  }

  public void disableAutoTrack() {
    this.autoTrackEnabled = false;
  }

  public boolean isAutoTrackEnabled() {
    return this.autoTrackEnabled;
  }

  public void updateLimelightPose(){
      LLResult result = limelight3A.getLatestResult();
      if (result != null && result.isValid()){
          Tx = result.getTx();
      }

  }

  }

