package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.pedroPathing.PedroConstants;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotHardware extends WSubsystem {
  private Follower m_follower;
  private Launcher launcher;
  private drive drive;
  private HardwareMap hardwareMap;

  public robotHardware(HardwareMap hardwareMap) {
    this.hardwareMap = hardwareMap;
    this.m_follower = PedroConstants.createFollower(hardwareMap);
    this.launcher = new Launcher(hardwareMap);
    this.drive = new drive(m_follower, hardwareMap);
  }

  @Override
  public void read() {
    launcher.read();
    drive.read();
  }

  @Override
  public void loop() {
    launcher.loop();
    drive.loop();
  }

  @Override
  public void write() {
    launcher.write();
    drive.write();
  }

  public Follower getFollower() {
    return m_follower;
  }
}
