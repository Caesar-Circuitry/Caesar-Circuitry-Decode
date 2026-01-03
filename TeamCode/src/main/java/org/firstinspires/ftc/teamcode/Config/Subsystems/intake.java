package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Constants;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Config.Utils.CurrentSensor;

import org.firstinspires.ftc.teamcode.Config.Utils.BeamBreak;

public class intake extends WSubsystem{
    private DcMotorEx intake, transfer;
    private CRServo block;
    private double intakePower;
    private double transferPower;
    private double blockPosition;
    private BeamBreak intakeBeamBreak;
    private BeamBreak transferBeamBreak;

    private CurrentSensor intakeCurrentSensor;
    private CurrentSensor transferCurrentSensor;
    private boolean isIntakeBeamBroken = false;
    private boolean isTransferBeamBroken = false;

    // previous-value guards to avoid redundant hardware writes
    // replaced NaN/compare approach with a simple boolean-first-write + != checks
    private double prevIntakePower = 0.0;
    private double prevTransferPower = 0.0;
    private double prevBlockPosition = 0.0;
    private boolean prevValuesInitialized = false;

    public intake(HardwareMap hwMap) {
        this.intake = hwMap.get(DcMotorEx.class, Constants.intake.INTAKE_MOTOR);
        // set motor direction from constants
        this.intake.setDirection(Constants.intake.INTAKE_MOTOR_DIRECTION);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.transfer = hwMap.get(DcMotorEx.class, Constants.intake.TRANSFER_MOTOR);
        this.transfer.setDirection(Constants.intake.TRANSFER_MOTOR_DIRECTION);
        this.transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.block = hwMap.get(CRServo.class, Constants.intake.FEEDER_SERVO);

        intakeBeamBreak = new BeamBreak(hwMap, Constants.intake.intakeBeamBreak);
        transferBeamBreak = new BeamBreak(hwMap, Constants.intake.transferBeamBreak);

        intakeCurrentSensor = new CurrentSensor(this.intake, Constants.intake.intakeOverCurrent);
        transferCurrentSensor =
                new CurrentSensor(this.transfer, Constants.intake.transferOverCurrent);
    }
    public enum DriveMode {
        TELEOP,
        AUTO
    }
    private enum intakeState {
        Ground,
        Human,
        Idle,
        Launch;
  }
    private intakeState state = intakeState.Idle;
    @Override
    public void read() {
        isIntakeBeamBroken = intakeBeamBreak.isBeamBroken();
        isTransferBeamBroken = transferBeamBreak.isBeamBroken();
        intakeCurrentSensor.readCurrent();
        transferCurrentSensor.readCurrent();
    }

    @Override
    public void loop() {
        switch (state){
            case Idle:
                this.blockPosition = Constants.intake.FEEDER_SERVO_CLOSE;
                this.intakePower = Constants.intake.INTAKE_MOTOR_HOLD;
                this.transferPower = Constants.intake.TRANSFER_MOTOR_HOLD;
                break;
            case Ground:
                this.blockPosition = Constants.intake.FEEDER_SERVO_CLOSE;
                this.intakePower = Constants.intake.INTAKE_MOTOR_FORWARD;
                this.transferPower = Constants.intake.TRANSFER_MOTOR_FORWARD;
                break;
            case Human:
                this.blockPosition = Constants.intake.FEEDER_SERVO_OPEN;
                this.intakePower = Constants.intake.INTAKE_MOTOR_HP;
                this.transferPower = Constants.intake.TRANSFER_MOTOR_HP;
                break;
            case Launch:
                this.blockPosition = Constants.intake.FEEDER_SERVO_OPEN;
                this.intakePower = Constants.intake.INTAKE_MOTOR_FORWARD;
                this.transferPower = Constants.intake.TRANSFER_MOTOR_FORWARD;
                break;
        }
    }

    @Override
    public void write() {
        // if this is the first write, force all writes
        if (!prevValuesInitialized) {
            this.intake.setPower(intakePower);
            this.transfer.setPower(transferPower);
            this.block.setPower(blockPosition);

            prevIntakePower = intakePower;
            prevTransferPower = transferPower;
            prevBlockPosition = blockPosition;
            prevValuesInitialized = true;
            return;
        }

        // subsequent writes: only update hardware when a value actually changed
        if (intakePower != prevIntakePower) {
            this.intake.setPower(intakePower);
            prevIntakePower = intakePower;
        }
        if (transferPower != prevTransferPower) {
            this.transfer.setPower(transferPower);
            prevTransferPower = transferPower;
        }
        if (blockPosition != prevBlockPosition) {
            this.block.setPower(blockPosition);
            prevBlockPosition = blockPosition;
        }
    }

}
