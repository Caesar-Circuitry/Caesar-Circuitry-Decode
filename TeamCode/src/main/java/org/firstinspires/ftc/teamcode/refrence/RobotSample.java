package org.firstinspires.ftc.teamcode.refrence;

import com.seattlesolvers.solverslib.command.Robot;

public class RobotSample extends Robot {
  public enum OpModeType {
    TELEOP,
    AUTO
  }

  // the constructor with a specified opmode type
  public RobotSample(OpModeType type) {
    if (type == OpModeType.TELEOP) {
      initTele();
    } else {
      initAuto();
    }
  }

  /*
   * Initialize teleop or autonomous, depending on which is used
   */
  public void initTele() {
    // initialize teleop-specific scheduler
  }

  public void initAuto() {
    // initialize auto-specific scheduler
  }
}
