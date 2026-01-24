package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.BeamBreak;
import org.firstinspires.ftc.teamcode.Config.Utils.CurrentSensor;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.InstantCommand;

import java.util.LinkedList;

public class Intake extends WSubsystem {
  // this is the Intake/Transfer class
  private DcMotorEx intakeMotor;
  private DcMotorEx transferMotor;

  private Servo FeederServo;

  private BeamBreak intakeBeamBreak;
  private BeamBreak transferBeamBreak;

  private CurrentSensor intakeCurrentSensor;
  private CurrentSensor transferCurrentSensor;

  private double intakeMotorTargetPower = 0;
  private double transferMotorTargetPower = 0;
  private double FeederServoTargetPos = Constants.Intake.FEEDER_SERVO_CLOSE;
  private double prevIntakeMotorTargetPower = 0;
  private double prevTransferMotorTargetPower = 0;
  private double prevFeederServoTargetPos = Constants.Intake.FEEDER_SERVO_CLOSE;

  private boolean isIntakeBeamBroken = false;
  private boolean isTransferBeamBroken = false;

  private enum State {
    GROUND_INTAKING,
    HP_INTAKING,
    LAUNCH,
    HOLD
  }

  private State intakeState = State.HOLD;

  private final LinkedList<TelemetryPacket> telemetryPackets = new LinkedList<>();

  public Intake(HardwareMap hardwareMap) {
    intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.Intake.INTAKE_MOTOR);
    transferMotor = hardwareMap.get(DcMotorEx.class, Constants.Intake.TRANSFER_MOTOR);

    intakeMotor.setDirection(Constants.Intake.INTAKE_MOTOR_DIRECTION);
    transferMotor.setDirection(Constants.Intake.TRANSFER_MOTOR_DIRECTION);

    FeederServo = hardwareMap.get(Servo.class, Constants.Intake.FEEDER_SERVO);

    intakeBeamBreak = new BeamBreak(hardwareMap, Constants.Intake.intakeBeamBreak, false);
    transferBeamBreak = new BeamBreak(hardwareMap, Constants.Intake.transferBeamBreak, false);

    intakeCurrentSensor = new CurrentSensor(this.intakeMotor, Constants.Intake.intakeOverCurrent);
    transferCurrentSensor =
        new CurrentSensor(this.transferMotor, Constants.Intake.transferOverCurrent);
  }

  @Override
  public void read() {
    isIntakeBeamBroken = intakeBeamBreak.isBeamBroken();
    isTransferBeamBroken = transferBeamBreak.isBeamBroken();
    intakeCurrentSensor.readCurrent();
    transferCurrentSensor.readCurrent();
  }

  @Override
  public void loop() {
    switch (intakeState) {
      case GROUND_INTAKING:
        this.FeederServoTargetPos = Constants.Intake.FEEDER_SERVO_CLOSE;
        this.intakeMotorTargetPower = Constants.Intake.INTAKE_MOTOR_FORWARD;
        this.transferMotorTargetPower = Constants.Intake.TRANSFER_MOTOR_FORWARD;
        break;
      case HOLD:
        this.FeederServoTargetPos = Constants.Intake.FEEDER_SERVO_CLOSE;
        this.intakeMotorTargetPower = Constants.Intake.INTAKE_MOTOR_HOLD;
        this.transferMotorTargetPower = Constants.Intake.TRANSFER_MOTOR_HOLD;
        break;
      case LAUNCH:
        this.FeederServoTargetPos = Constants.Intake.FEEDER_SERVO_OPEN;
        this.intakeMotorTargetPower = Constants.Intake.INTAKE_MOTOR_FORWARD;
        this.transferMotorTargetPower = Constants.Intake.TRANSFER_MOTOR_FORWARD;
        break;
      case HP_INTAKING:
        this.FeederServoTargetPos = Constants.Intake.FEEDER_SERVO_OPEN;
        this.intakeMotorTargetPower = Constants.Intake.INTAKE_MOTOR_HP;
        this.transferMotorTargetPower = Constants.Intake.TRANSFER_MOTOR_HP;
        break;
    }

    // Telemetry logging
    if (Constants.Intake.logTelemetry) {
      telemetryPackets.clear();

      // State and targets
      telemetryPackets.add(new TelemetryPacket("State", intakeState.name()));
      telemetryPackets.add(new TelemetryPacket("Feeder Target", FeederServoTargetPos));
      telemetryPackets.add(new TelemetryPacket("Intake Target Power", intakeMotorTargetPower));
      telemetryPackets.add(new TelemetryPacket("Transfer Target Power", transferMotorTargetPower));

      // Sensor readings
      telemetryPackets.add(new TelemetryPacket("Intake Beam Broken", isIntakeBeamBroken));
      telemetryPackets.add(new TelemetryPacket("Transfer Beam Broken", isTransferBeamBroken));
      telemetryPackets.add(new TelemetryPacket("Intake Current(A)", intakeCurrentSensor.getCurrent()));
      telemetryPackets.add(new TelemetryPacket("Transfer Current(A)", transferCurrentSensor.getCurrent()));
    }
  }

  @Override
  public void write() {
    if (FeederServoTargetPos != prevFeederServoTargetPos) {
      this.FeederServo.setPosition(FeederServoTargetPos);
    }
    if (intakeMotorTargetPower != prevIntakeMotorTargetPower) {
      this.intakeMotor.setPower(intakeMotorTargetPower);
    }
    if (transferMotorTargetPower != prevTransferMotorTargetPower) {
      this.transferMotor.setPower(transferMotorTargetPower);
    }

    this.prevFeederServoTargetPos = FeederServoTargetPos;
    this.prevIntakeMotorTargetPower = intakeMotorTargetPower;
    this.prevTransferMotorTargetPower = transferMotorTargetPower;
  }

  public InstantCommand GroundIntake(){
      return new InstantCommand(()-> intakeState = State.GROUND_INTAKING);
  }
  public InstantCommand Hold(){
      return new InstantCommand(()-> intakeState = State.HOLD);
  }
  public InstantCommand Launch(){
      return new InstantCommand(()-> intakeState = State.LAUNCH);
  }
  public InstantCommand HP_Intaking(){
      return new InstantCommand(()-> intakeState = State.HP_INTAKING);
  }

  @Override
  public LinkedList<TelemetryPacket> getTelemetry() {
    return telemetryPackets;
  }
}
