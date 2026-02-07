package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueStartFromFar {
    // Unified pose points with headings (in radians)
    private static final Pose START_POSE = new Pose(61.000, 7.000, Math.toRadians(180));
    private static final Pose ARTIFACT_0_POSE = new Pose(23.791, 59.614, Math.toRadians(180));
    private static final Pose LAUNCH_POSE = new Pose(52.000, 79.000, Math.toRadians(220));
    private static final Pose ARTIFACT_1_POSE = new Pose(23.791, 35.579, Math.toRadians(180));
    private static final Pose RAMP_APPROACH_POSE = new Pose(18.000, 69.000, Math.toRadians(180));
    private static final Pose RAMP_INTAKE_POSE = new Pose(9.000, 58.000, Math.toRadians(100));

    // Control points for curves (no heading needed)
    private static final Pose INTAKE_0_CONTROL = new Pose(48.471, 63.679);
    private static final Pose INTAKE_1_CONTROL = new Pose(43.346, 30.552);

    public PathChain Launch0;
    public PathChain Intake0;
    public PathChain Launch1;
    public PathChain Intake1;
    public PathChain Launch2;
    public PathChain moveToRamp0;
    public PathChain IntakeRamp0;
    public PathChain Launch3;
    public PathChain moveToRamp1;
    public PathChain IntakeRamp1;
    public PathChain Launch4;
    public PathChain moveToRamp2;
    public PathChain IntakeRamp2;
    public PathChain Launch5;

    public BlueStartFromFar(Follower follower) {
        Launch0 = follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, START_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), START_POSE.getHeading())
                .setReversed()
                .build();

        Intake0 = follower.pathBuilder().addPath(
                        new BezierCurve(START_POSE, INTAKE_0_CONTROL, ARTIFACT_0_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), ARTIFACT_0_POSE.getHeading())
                .build();

        Launch1 = follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_0_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(ARTIFACT_0_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE, INTAKE_1_CONTROL, ARTIFACT_1_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), ARTIFACT_1_POSE.getHeading())
                .build();

        Launch2 = follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_1_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(ARTIFACT_1_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        moveToRamp0 = follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();

        IntakeRamp0 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_APPROACH_POSE, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();

        Launch3 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        moveToRamp1 = follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();

        IntakeRamp1 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_APPROACH_POSE, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();

        Launch4 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        moveToRamp2 = follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();

        IntakeRamp2 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_APPROACH_POSE, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();

        Launch5 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }
}
