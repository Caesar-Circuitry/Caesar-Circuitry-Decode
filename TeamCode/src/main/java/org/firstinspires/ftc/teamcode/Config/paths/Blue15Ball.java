package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;

public class Blue15Ball {
    // Unified pose points with headings (in radians)
    public static final Pose START_POSE = new Pose(32,135, Math.toRadians(270));
    public static final Pose LAUNCH_POSE_ANGLE = new Pose(57,78, Math.toRadians(220)); //57 78
    public static final Pose LAUNCH_POSE_STRAIGHT = new Pose(44,73, Math.toRadians(190));
    public static final Pose INTAKE_MIDDLE = new Pose(6,50,Math.toRadians(180));
    public static final Pose INTAKE_TOP = new Pose(13,73,Math.toRadians(180));
    public static final Pose GATE_HANDLE = new Pose(20,60,Math.toRadians(180));
    public static final Pose RAMP_INTAKE = new Pose(-10,47,Math.toRadians(128));//135
    public static final Pose READJUST_FINAL = new Pose(0,47,Math.toRadians(128));
    public static final Pose FINAL_LAUNCH = new Pose(40,120,Math.toRadians(327));

    // Control points for curves (no heading needed)
    public static final Pose CONTROL_LAUNCH = new Pose(45,46);
    public static final Pose CONTROL_GATE = new Pose(35,65);

    private Follower follower;

    public Blue15Ball(Follower follower) {
        this.follower = follower;
    }

    public PathChain moveTo1stLaunch() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, LAUNCH_POSE_ANGLE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), LAUNCH_POSE_ANGLE.getHeading())
                .build();
    }
    public PathChain moveToIntakeMid() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE_ANGLE, CONTROL_LAUNCH, INTAKE_MIDDLE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_ANGLE.getHeading(), INTAKE_MIDDLE.getHeading())
                .build();
    }
    public PathChain moveTo2ndLaunch(Intake intake) {
        return follower.pathBuilder().addPath(
                        new BezierCurve(INTAKE_MIDDLE, CONTROL_LAUNCH, LAUNCH_POSE_STRAIGHT)
                ).setLinearHeadingInterpolation(INTAKE_MIDDLE.getHeading(), LAUNCH_POSE_STRAIGHT.getHeading())
                .build();
    }
    public PathChain moveToIntakeTop() {
        return follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE_STRAIGHT, INTAKE_TOP)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_STRAIGHT.getHeading(), INTAKE_TOP.getHeading())
                .build();
    }
    public PathChain moveTo3rdLaunch() {
        return follower.pathBuilder().addPath(
                        new BezierLine(INTAKE_TOP, new Pose(LAUNCH_POSE_ANGLE.getX()-6,LAUNCH_POSE_ANGLE.getY()-2))
                ).setLinearHeadingInterpolation(INTAKE_TOP.getHeading(), LAUNCH_POSE_ANGLE.getHeading())
                .setGlobalDeceleration(0.25)
                .build();
    }
    public PathChain moveToGate() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE_ANGLE, CONTROL_GATE, GATE_HANDLE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_ANGLE.getHeading(), GATE_HANDLE.getHeading())
                .build();
    }
    public PathChain moveToRampIntake() {
        return follower.pathBuilder().addPath(
                        new BezierLine(GATE_HANDLE, RAMP_INTAKE)
                ).setLinearHeadingInterpolation(GATE_HANDLE.getHeading(), RAMP_INTAKE.getHeading())
                .setTValueConstraint(.4)
                .build();
    }
    public PathChain moveTo4thLaunch() {
        return follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE, new Pose(LAUNCH_POSE_ANGLE.getX()-1,LAUNCH_POSE_ANGLE.getY()+1))
                ).setLinearHeadingInterpolation(RAMP_INTAKE.getHeading(), LAUNCH_POSE_ANGLE.getHeading())
                .build();
    }
    public PathChain readjust() {
        return follower.pathBuilder().addPath(
                new BezierLine(RAMP_INTAKE,READJUST_FINAL)
        ).setLinearHeadingInterpolation(RAMP_INTAKE.getHeading(), READJUST_FINAL.getHeading())
                .build();
    }
    public PathChain moveTo5thLaunch() {
        return follower.pathBuilder().addPath(
                        new BezierLine(READJUST_FINAL, FINAL_LAUNCH)
                ).setLinearHeadingInterpolation(READJUST_FINAL.getHeading(), FINAL_LAUNCH.getHeading())
                .build();
    }
}