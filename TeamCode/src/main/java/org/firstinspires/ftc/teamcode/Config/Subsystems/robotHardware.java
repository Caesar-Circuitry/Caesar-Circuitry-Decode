package org.firstinspires.ftc.teamcode.Config.Subsystems;

import java.util.List;

import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware extends WSubsystem {
  private Follower m_follower;
  private Launcher launcher;
  private HardwareMap hardwareMap;
  List<LynxModule> hubs;

  public robotHardware(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
    this.hubs = hardwareMap.getAll(LynxModule.class);
    hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    this.m_follower = PedroConstants.createFollower(hardwareMap);
    this.launcher = new Launcher(hardwareMap);
  }

  @Override
  public void read() {
    launcher.read();
  }

  @Override
  public void loop() {
    launcher.loop();
  }

  @Override
  public void write() {
    launcher.write();
    hubs.forEach(LynxModule::clearBulkCache);
  }

  public Follower getFollower() {
    return m_follower;
  }
}
