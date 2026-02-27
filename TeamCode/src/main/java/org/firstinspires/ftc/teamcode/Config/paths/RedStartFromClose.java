package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedStartFromClose {
    // Unified pose points with headings (mirrored from BlueStartFromClose)
    public static final Pose START_POSE = BlueStartFromClose.START_POSE.mirror();
    public static final Pose LAUNCH_POSE_0 = BlueStartFromClose.LAUNCH_POSE_0.mirror();
    public static final Pose ARTIFACT_0_POSE = BlueStartFromClose.ARTIFACT_0_POSE.mirror();
    public static final Pose LAUNCH_POSE = BlueStartFromClose.LAUNCH_POSE.mirror();
    public static final Pose LAUNCH_POSE_220 = BlueStartFromClose.LAUNCH_POSE_220.mirror();
    public static final Pose ARTIFACT_1_POSE = BlueStartFromClose.ARTIFACT_1_POSE.mirror();
    public static final Pose RAMP_APPROACH_POSE = BlueStartFromClose.RAMP_APPROACH_POSE.mirror();
    public static final Pose RAMP_INTAKE_POSE = BlueStartFromClose.RAMP_INTAKE_POSE.mirror();

    // Control points for curves (mirrored from BlueStartFromClose)
    public static final Pose INTAKE_ARTIFACT_0_CONTROL = BlueStartFromClose.INTAKE_ARTIFACT_0_CONTROL.mirror();
    public static final Pose INTAKE_ARTIFACT_1_CONTROL = BlueStartFromClose.INTAKE_ARTIFACT_1_CONTROL.mirror();
    public static final Pose RAMP_0_CONTROL = BlueStartFromClose.RAMP_0_CONTROL.mirror();
    public static final Pose INTAKE_RAMP_CONTROL = BlueStartFromClose.INTAKE_RAMP_CONTROL.mirror();

    private final Follower follower;

    public RedStartFromClose(Follower follower) {
        this.follower = follower;
    }

    public PathChain moveToLaunch0() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, LAUNCH_POSE_0)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), LAUNCH_POSE_0.getHeading())
                .setReversed()
                .build();
    }

    public PathChain intakeArtifact0() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE_0, INTAKE_ARTIFACT_0_CONTROL, ARTIFACT_0_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_0.getHeading(), ARTIFACT_0_POSE.getHeading())
                .build();
    }

    public PathChain moveToLaunch1() {
        return follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_0_POSE, LAUNCH_POSE_220)
                ).setLinearHeadingInterpolation(ARTIFACT_0_POSE.getHeading(), LAUNCH_POSE_220.getHeading())
                .build();
    }

    public PathChain intakeArtifact1() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE_220, INTAKE_ARTIFACT_1_CONTROL, ARTIFACT_1_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_220.getHeading(), ARTIFACT_1_POSE.getHeading())
                .build();
    }

    public PathChain moveToLaunch2() {
        return follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_1_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(ARTIFACT_1_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }

    public PathChain moveToRamp0() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE, RAMP_0_CONTROL, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();
    }

    public PathChain intakeRamp0() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();
    }

    public PathChain moveToLaunch3() {
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

    public PathChain moveToLaunch4() {
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

    public PathChain moveToLaunch5() {
        return follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }
}
