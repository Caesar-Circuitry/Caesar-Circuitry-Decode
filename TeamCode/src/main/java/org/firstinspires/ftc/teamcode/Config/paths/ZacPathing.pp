package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class ZacPathing {
    // Unified pose points with headings (in radians)
    public static final Pose START_POSE = new Pose(32,135, Math.toRadians(-90));
    public static final Pose LAUNCH1_POSE = new Pose(55,84, Math.toRadians(220));
    public static final Pose LAUNCH2_POSE = new Pose(55,84, Math.toRadians(180));
    public static final Pose INTAKE_MIDDLE = new Pose(22,60,Math.toRadians(180));
    public static final Pose INTAKE_TOP = new Pose(22,84,Math.toRadians(180));

    // Control points for curves (no heading needed)
    public static final Pose CONTROL_LAUNCH = new Pose(50,58);

    private Follower follower;

    public ZacPathing(Follower follower) {
        this.follower = follower;
    }

    public PathChain moveToLaunch() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, LAUNCH1_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), LAUNCH1_POSE.getHeading())
                .build();
    }
    public PathChain moveToIntakeMid() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH1_POSE, CONTROL_LAUNCH, INTAKE_MIDDLE)
                ).setLinearHeadingInterpolation(LAUNCH1_POSE.getHeading(), INTAKE_MIDDLE.getHeading())
                .build();
    }
    public PathChain moveToLaunchMid() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(INTAKE_MIDDLE, CONTROL_LAUNCH, LAUNCH2_POSE)
                ).setLinearHeadingInterpolation(INTAKE_MIDDLE.getHeading(), LAUNCH2_POSE.getHeading())
                .build();
    }
    public PathChain moveToIntakeTop() {
        return follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH2_POSE, INTAKE_TOP)
                ).setLinearHeadingInterpolation(LAUNCH2_POSE.getHeading(), INTAKE_TOP.getHeading())
//                .setReversed()
                .build();
    }
    public PathChain moveToLaunchTop() {
        return follower.pathBuilder().addPath(
                        new BezierLine(INTAKE_TOP, LAUNCH2_POSE)
                ).setLinearHeadingInterpolation(INTAKE_TOP.getHeading(), LAUNCH2_POSE.getHeading())
//                .setReversed()
                .build();
    }

}
