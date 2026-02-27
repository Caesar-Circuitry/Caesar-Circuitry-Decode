package org.firstinspires.ftc.teamcode.refrence;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Robot;

public class CommandSample extends CommandBase {
  private Robot robot;

  public CommandSample(Robot robot) {
    // declare subsystem dependencies here EX: addRequirements(subsystem);
    this.robot = robot;
  }

  @Override
  public void initialize() {
    // ran once when the command is started
  }

  @Override
  public void execute() {
    // ran repeatedly until the command ends
  }

  @Override
  public void end(boolean interrupted) {
    // ran once when the command ends
  }

  @Override
  public boolean isFinished() {
    // return true when the command should end
    return false;
  }
}
