package org.firstinspires.ftc.teamcode.Config.Subsystems;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware extends WSubsystem {
  private Follower m_follower;
  private Launcher Launcher;
  private Intake Intake;
//  private Turret Turret;
//  private Vision vision;
  private HardwareMap hardwareMap;
  List<LynxModule> hubs;

  public robotHardware(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
    this.hubs = hardwareMap.getAll(LynxModule.class);
    hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    this.m_follower = PedroConstants.createFollower(hardwareMap);
    this.m_follower.setStartingPose(new Pose(0,0,0));
    this.Launcher = new Launcher(hardwareMap);
    this.Intake = new Intake(hardwareMap);
//    this.Turret = new Turret(hardwareMap,m_follower);
//    this.vision = new Vision(hardwareMap,m_follower,Turret);

  }

  @Override
  public void read() {
    Launcher.read();
    Intake.read();
//    Turret.read();
//    vision.read();
  }

  @Override
  public void loop() {
    Launcher.loop();
    Intake.loop();
    m_follower.update();
//    Turret.loop();
//    vision.loop();
  }

  @Override
  public void write() {
    Launcher.write();
    Intake.write();
//    Turret.write();
//    vision.write();
    hubs.forEach(LynxModule::clearBulkCache);

  }

  public Follower getFollower() {
    return m_follower;
  }
  public Launcher getLauncher(){
      return Launcher;
  }
  public Intake getIntake(){
      return Intake;
  }
//  public Turret getTurret(){
//      return Turret;
//  }
//  public Vision getVision(){
//      return vision;
//  }
}
