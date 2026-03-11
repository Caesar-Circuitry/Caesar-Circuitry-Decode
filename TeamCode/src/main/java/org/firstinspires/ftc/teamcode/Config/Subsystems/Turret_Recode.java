package org.firstinspires.ftc.teamcode.Config.Subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config.Constants;
import org.firstinspires.ftc.teamcode.Config.Utils.TelemetryPacket;

import java.util.LinkedList;

public class Turret_Recode extends WSubsystem{
    /**
     * Going to Switch to using Radians rather than degrees
     * Pedro Coordinates turret increases counterclockwise
     * */
    private CRServoGroup turretServos;
    private Motor.Encoder turretEncoder;
    private AbsoluteAnalogEncoder turretAbsoluteEncoder;
    private SquIDFController turretController;
    private double currentTargetAngle = 0.0;

    public enum TurretState{
        TRACK_GOAL,
        TRACK_ANGLE,
        OFF
    }

    public TurretState turretState = TurretState.TRACK_GOAL;


    public Turret_Recode(HardwareMap hmap){
        CRServoEx servoLead = new CRServoEx(hmap,"servo1");
        CRServoEx servoFollow = new CRServoEx(hmap,"servo2");
        turretServos = new CRServoGroup(servoLead,servoFollow);
        turretEncoder = new Motor(hmap, "encoder").encoder;
        turretAbsoluteEncoder = new AbsoluteAnalogEncoder(hmap,"absoluteEncoder");
        turretController = new SquIDFController(0,0,0,0);
    }

    @Override
    public void read() {

    }

    @Override
    public void loop() {
        switch (turretState){
            case OFF:
                break;
            case TRACK_GOAL:
                //trackGoal
                break;
            case TRACK_ANGLE:
                break;
        }
    }

    @Override
    public void write() {

    }

    @Override
    public LinkedList<TelemetryPacket> getTelemetry() {
        return null;
    }

    public double getTurretAngle(){
        double RadiansPerTick = Constants.Turret.RadiansPerTick;
        return MathUtils.normalizeRadians(turretEncoder.getPosition() * RadiansPerTick,false);
    }
    public void resetTurretEncoder(){
        //TODO: set the offset for the throughbore encoder based on absolute encoder data
    }
    /**
     * @param robotPose robotPose in PedroCoordinates
     * @return Limelight Heading in radians
     * */
    public double getLimelightHeading(Pose robotPose){
        return getTurretAngle() + robotPose.getHeading();
    }

    private void Track_Goal(Pose robotPose, Vector robotVelocity, double artifactSpeed){
        Pose pose = robotPose;
        Vector velo = robotVelocity;
        double artifact_Speed = artifactSpeed;

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double vx = velo.getXComponent();
        double vy = velo.getYComponent();

        double dx = Constants.Robot.Goal.getX() - robotX;
        double dy = Constants.Robot.Goal.getY() - robotY;

        double distance = Math.sqrt(dx*dx + dy*dy);

        double ux = dx / distance;
        double uy = dy / distance;

        double shotVX = ux * artifact_Speed;
        double shotVY = uy * artifact_Speed;

        double compensatedVX = shotVX - vx;
        double compensatedVY = shotVY - vy;

        double fieldAngle = Math.atan2(compensatedVY,compensatedVX);

        double desiredTurretAngle = MathUtils.normalizeAngle(fieldAngle-robotHeading, false, AngleUnit.RADIANS);

        double plannedTarget = planTurretTarget(currentTargetAngle,desiredTurretAngle);
        turretController.setSetPoint(plannedTarget);

    }

    private double planTurretTarget(double current, double desired){
        double TurretMin = Constants.Turret.TurretMin;
        double TurretMax = Constants.Turret.TurretMax;

        double WrapAroundZone = Constants.Turret.WrapAroundZone;

        double desiredAngle = MathUtils.clamp(desired,TurretMin, TurretMax);
        double currentAngle = current;

        if (currentAngle < 0 && desiredAngle >0){
            if (Math.abs(currentAngle)>WrapAroundZone && Math.abs(desiredAngle) > WrapAroundZone){
                return 0.0;
            }
        }

        if (currentAngle > 0 && desiredAngle <0){
            if (Math.abs(currentAngle)>WrapAroundZone && Math.abs(desiredAngle) > WrapAroundZone){
                return 0.0;
            }
        }

        return desiredAngle;

    }

}
