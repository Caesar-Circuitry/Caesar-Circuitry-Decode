package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueStartFromFar {
    // Unified pose points with headings (in radians)
    public static final Pose START_POSE = new Pose(61.000, 7.000, Math.toRadians(180));
    public static final Pose ARTIFACT_0_POSE = new Pose(23.791, 59.614, Math.toRadians(180));
    public static final Pose LAUNCH_POSE = new Pose(52.000, 79.000, Math.toRadians(220));
    public static final Pose ARTIFACT_1_POSE = new Pose(23.791, 35.579, Math.toRadians(180));
    public static final Pose RAMP_APPROACH_POSE = new Pose(18.000, 69.000, Math.toRadians(180));
    public static final Pose RAMP_INTAKE_POSE = new Pose(9.000, 58.000, Math.toRadians(100));

    // Control points for curves (no heading needed)
    public static final Pose INTAKE_0_CONTROL = new Pose(48.471, 63.679);
    public static final Pose INTAKE_1_CONTROL = new Pose(43.346, 30.552);
    public static final Pose INTAKE_RAMP_CONTROL = new Pose(18.863, 57.418);

    private final Follower follower;

    public BlueStartFromFar(Follower follower) {
        this.follower = follower;
    }

    public PathChain launch0() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, START_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), START_POSE.getHeading())
                .setReversed()
                .build();
    }

    public PathChain intake0() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(START_POSE, INTAKE_0_CONTROL, ARTIFACT_0_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), ARTIFACT_0_POSE.getHeading())
                .build();
    }

    public PathChain launch1() {
        return follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_0_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(ARTIFACT_0_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }

    public PathChain intake1() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE, INTAKE_1_CONTROL, ARTIFACT_1_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), ARTIFACT_1_POSE.getHeading())
                .build();
    }

    public PathChain launch2() {
        return follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_1_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(ARTIFACT_1_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }

    public PathChain moveToRamp0() {
        return follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();
    }

    public PathChain intakeRamp0() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();
    }

    public PathChain launch3() {
        return follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }

    public PathChain moveToRamp1() {
        return follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();
    }

    public PathChain intakeRamp1() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();
    }

    public PathChain launch4() {
        return follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }

    public PathChain moveToRamp2() {
        return follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();
    }

    public PathChain intakeRamp2() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();
    }

    public PathChain launch5() {
        return follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }
}
