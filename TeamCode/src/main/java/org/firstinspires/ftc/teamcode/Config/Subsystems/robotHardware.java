package org.firstinspires.ftc.teamcode.Config.Subsystems;

import java.util.LinkedList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware extends WSubsystem {
  private Drivetrain drivetrain;
  private Launcher Launcher;
  private Intake Intake;
  private Turret Turret;
  //private Vision vision;
  private HardwareMap hardwareMap;
  List<LynxModule> hubs;
  private LinkedList<TelemetryPacket> telemetry;

  public robotHardware(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
    this.hubs = hardwareMap.getAll(LynxModule.class);
    hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    this.drivetrain = new Drivetrain(hardwareMap);
    this.Launcher = new Launcher(hardwareMap);
    this.Intake = new Intake(hardwareMap);
    this.Turret = new Turret(hardwareMap,drivetrain.getFollower(),getLauncher());
    //this.vision = new Vision(hardwareMap,drivetrain.getFollower(),Turret);
    this.telemetry = new LinkedList<TelemetryPacket>();

  }

  @Override
  public void read() {
    drivetrain.read();
    Launcher.read();
    Intake.read();
    Turret.read();
//    vision.read();
  }

  @Override
  public void loop() {
    drivetrain.loop();
    Launcher.loop();
    Intake.loop();
    Turret.loop();
//    vision.loop();
  }

  @Override
  public void write() {
    drivetrain.write();
    Launcher.write();
    Intake.write();
    Turret.write();
//    vision.write();
    hubs.forEach(LynxModule::clearBulkCache);

  }

    @Override
    public LinkedList<TelemetryPacket> getTelemetry() {
        telemetry.clear();
        telemetry.addAll(drivetrain.getTelemetry());
        telemetry.addAll(Launcher.getTelemetry());
        telemetry.addAll(Intake.getTelemetry());
        telemetry.addAll(Turret.getTelemetry());
//        telemetry.addAll(vision.getTelemetry());
        return telemetry;
    }

    public Follower getFollower() {
    return drivetrain.getFollower();
  }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Launcher getLauncher(){
      return Launcher;
  }
  public Intake getIntake(){
      return Intake;
  }
  public Turret getTurret(){
      return Turret;
  }
//  public Vision getVision(){
//      return vision;
//  }
}
