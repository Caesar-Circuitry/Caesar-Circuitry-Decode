package org.firstinspires.ftc.teamcode.Config.Subsystems;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import java.util.LinkedList;
import java.util.List;

public class Launcher_Recode extends WSubsystem {

  private MotorGroup flywheel;

  private InterpLUT flywheelSpeeds = new InterpLUT();

  private double targetVelocity = 0; // Ticks per second

  public enum LauncherState{
    LAUNCH_CLOSE,
    LAUNCH_FAR,
    HUMAN_PLAYER,
    LAUNCH_ODOMETRY,
    IDLE,
    NO_POWER,
    STOP_WITH_POWER
  }

  private final LinkedList<TelemetryPacket> telemetryPackets = new LinkedList<>();

  public Launcher_Recode(HardwareMap hardwareMap) {
    MotorEx flywheelLead = new MotorEx(hardwareMap, Constants.Launcher.FLYWHEEL_MOTOR_LEAD, Motor.GoBILDA.BARE);
    MotorEx flywheelFollow = new MotorEx(hardwareMap,Constants.Launcher.FLYWHEEL_MOTOR_FOLLOW, Motor.GoBILDA.BARE);

    flywheelFollow.setInverted(true);

    flywheel = new MotorGroup(flywheelLead,flywheelFollow);
    flywheel.setRunMode(Motor.RunMode.VelocityControl);
    flywheel.setVeloCoefficients(Constants.Launcher.kP,Constants.Launcher.kI,Constants.Launcher.kD);
    flywheel.setFeedforwardCoefficients(Constants.Launcher.kS,Constants.Launcher.Kv,Constants.Launcher.Ka);
  }

  @Override
  public void read() {
  }

  @Override
  public void loop() {
  }

  private void updateTelemetry() {
    if (Constants.Launcher.logTelemetry) {
    }
  }

  @Override
  public void write() {
    flywheel.set(0);
  }

  @Override
  public LinkedList<TelemetryPacket> getTelemetry() {
    return null;
  }
}
