package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public abstract class WSubsystem extends SubsystemBase {
  /** put all code that reads hardware in this function */
  public abstract void read();

  /** stores all non hardware accessing code in this function */
  public abstract void loop();

  /** put all code that writes to hardware in this function */
  public abstract void write();
}
