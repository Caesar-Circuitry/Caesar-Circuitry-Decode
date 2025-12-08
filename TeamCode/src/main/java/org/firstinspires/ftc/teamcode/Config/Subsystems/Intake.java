package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.BeamBreak;
import org.firstinspires.ftc.teamcode.Config.Utils.currentSensor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends WSubsystem {
  // this is the Intake/Transfer class
  private DcMotorEx intakeMotor;
  private DcMotorEx transferMotor;

  private Servo FeederServo;

  private BeamBreak intakeBeamBreak;
  private BeamBreak transferBeamBreak;

  private currentSensor intakeCurrentSensor;
  private currentSensor transferCurrentSensor;

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

  private State intakeState;

  private Intake(HardwareMap hardwareMap) {
    intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.Intake.INTAKE_MOTOR);
    transferMotor = hardwareMap.get(DcMotorEx.class, Constants.Intake.TRANSFER_MOTOR);

    intakeMotor.setDirection(Constants.Intake.INTAKE_MOTOR_DIRECTION);
    transferMotor.setDirection(Constants.Intake.TRANSFER_MOTOR_DIRECTION);

    FeederServo = hardwareMap.get(Servo.class, Constants.Intake.FEEDER_SERVO);

    intakeBeamBreak = new BeamBreak(hardwareMap, Constants.Intake.intakeBeamBreak);
    transferBeamBreak = new BeamBreak(hardwareMap, Constants.Intake.transferBeamBreak);

    intakeCurrentSensor = new currentSensor(this.intakeMotor, Constants.Intake.intakeOverCurrent);
    transferCurrentSensor =
        new currentSensor(this.transferMotor, Constants.Intake.transferOverCurrent);
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
}
