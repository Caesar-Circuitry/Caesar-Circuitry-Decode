package org.firstinspires.ftc.teamcode.refrence;

import org.firstinspires.ftc.teamcode.Config.Subsystems.WSubsystem;

public class SubsystemSample extends WSubsystem {

  // declare hardware here EX: private DcMotorEx motor;

  public SubsystemSample() {
    // initialize hardware here EX: motor = hardwareMap.get(DcMotorEx.class, "motorName");

  }

  @Override
  public void read() {
    // insert all hardware reading code here EX: motor.getCurrentPosition();

  }

  @Override
  public void loop() {
    // insert all non hardware accessing code here EX: calculations, state machines, etc

  }

  @Override
  public void write() {
    // insert all hardware writing code here EX: motor.setPower(power);

  }
}
