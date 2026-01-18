package org.firstinspires.ftc.teamcode.Config.Subsystems;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware extends WSubsystem {
  private Follower m_follower;
  private Launcher Launcher;
  private Intake Intake;
  private Turret Turret;
  private HardwareMap hardwareMap;
  List<LynxModule> hubs;

  public robotHardware(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
    this.hubs = hardwareMap.getAll(LynxModule.class);
    hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    this.m_follower = PedroConstants.createFollower(hardwareMap);
    this.Launcher = new Launcher(hardwareMap);
    this.Intake = new Intake(hardwareMap);

  }

  @Override
  public void read() {
    Launcher.read();
    Intake.read();
    Turret.read();
  }

  @Override
  public void loop() {
    Launcher.loop();
    Intake.loop();
    Turret.read();
  }

  @Override
  public void write() {
    Launcher.write();
    Intake.write();
    Turret.read();
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
  public Turret getTurret(){
      return Turret;
  }
}
