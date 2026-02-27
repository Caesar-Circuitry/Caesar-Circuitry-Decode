package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedStartFromFar {
    // Unified pose points with headings (mirrored from BlueStartFromFar)
    public static final Pose START_POSE = BlueStartFromFar.START_POSE.mirror();
    public static final Pose ARTIFACT_0_POSE = BlueStartFromFar.ARTIFACT_0_POSE.mirror();
    public static final Pose LAUNCH_POSE = BlueStartFromFar.LAUNCH_POSE.mirror();
    public static final Pose ARTIFACT_1_POSE = BlueStartFromFar.ARTIFACT_1_POSE.mirror();
    public static final Pose RAMP_APPROACH_POSE = BlueStartFromFar.RAMP_APPROACH_POSE.mirror();
    public static final Pose RAMP_INTAKE_POSE = BlueStartFromFar.RAMP_INTAKE_POSE.mirror();

    // Control points for curves (mirrored from BlueStartFromFar)
    public static final Pose INTAKE_0_CONTROL = BlueStartFromFar.INTAKE_0_CONTROL.mirror();
    public static final Pose INTAKE_1_CONTROL = BlueStartFromFar.INTAKE_1_CONTROL.mirror();
    public static final Pose INTAKE_RAMP_CONTROL = BlueStartFromFar.INTAKE_RAMP_CONTROL.mirror();

    private final Follower follower;

    public RedStartFromFar(Follower follower) {
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
