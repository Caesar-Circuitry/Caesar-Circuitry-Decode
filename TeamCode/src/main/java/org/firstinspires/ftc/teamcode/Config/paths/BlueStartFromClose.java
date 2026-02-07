package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueStartFromClose {
    // Unified pose points with headings (in radians)
    private static final Pose START_POSE = new Pose(16.021, 113.501, Math.toRadians(180));
    private static final Pose LAUNCH_POSE_0 = new Pose(47.772, 93.332, Math.toRadians(260));
    private static final Pose ARTIFACT_0_POSE = new Pose(25.142, 60.386, Math.toRadians(180));
    private static final Pose LAUNCH_POSE = new Pose(52.000, 79.000, Math.toRadians(200));
    private static final Pose LAUNCH_POSE_220 = new Pose(52.000, 79.000, Math.toRadians(220));
    private static final Pose ARTIFACT_1_POSE = new Pose(23.791, 83.807, Math.toRadians(180));
    private static final Pose RAMP_APPROACH_POSE = new Pose(18.000, 69.000, Math.toRadians(180));
    private static final Pose RAMP_INTAKE_POSE = new Pose(9.000, 58.000, Math.toRadians(100));

    // Control points for curves (no heading needed)
    private static final Pose INTAKE_ARTIFACT_0_CONTROL = new Pose(40.363, 60.590);
    private static final Pose INTAKE_ARTIFACT_1_CONTROL = new Pose(46.241, 85.758);
    private static final Pose RAMP_0_CONTROL = new Pose(51.076, 78.325);
    private static final Pose INTAKE_RAMP_CONTROL = new Pose(18.863, 57.418);

    public PathChain MoveToLaunch0;
    public PathChain IntakeArtifact0;
    public PathChain MoveToLaunch1;
    public PathChain IntakeArtifact1;
    public PathChain MoveToLaunch2;
    public PathChain MoveToRamp0;
    public PathChain IntakeRamp0;
    public PathChain MoveToLaunch3;
    public PathChain MoveToRamp1;
    public PathChain IntakeRamp1;
    public PathChain MoveToLaunch4;
    public PathChain MoveToRamp2;
    public PathChain IntakeRamp2;
    public PathChain MoveToLaunch5;

    public BlueStartFromClose(Follower follower) {
        MoveToLaunch0 = follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, LAUNCH_POSE_0)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), LAUNCH_POSE_0.getHeading())
                .setReversed()
                .build();

        IntakeArtifact0 = follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE_0, INTAKE_ARTIFACT_0_CONTROL, ARTIFACT_0_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_0.getHeading(), ARTIFACT_0_POSE.getHeading())
                .build();

        MoveToLaunch1 = follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_0_POSE, LAUNCH_POSE_220)
                ).setLinearHeadingInterpolation(ARTIFACT_0_POSE.getHeading(), LAUNCH_POSE_220.getHeading())
                .build();

        IntakeArtifact1 = follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE_220, INTAKE_ARTIFACT_1_CONTROL, ARTIFACT_1_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE_220.getHeading(), ARTIFACT_1_POSE.getHeading())
                .build();

        MoveToLaunch2 = follower.pathBuilder().addPath(
                        new BezierLine(ARTIFACT_1_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(ARTIFACT_1_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        MoveToRamp0 = follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE, RAMP_0_CONTROL, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();

        IntakeRamp0 = follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();

        MoveToLaunch3 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        MoveToRamp1 = follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();

        IntakeRamp1 = follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();

        MoveToLaunch4 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();

        MoveToRamp2 = follower.pathBuilder().addPath(
                        new BezierLine(LAUNCH_POSE, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();

        IntakeRamp2 = follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();

        MoveToLaunch5 = follower.pathBuilder().addPath(
                        new BezierLine(RAMP_INTAKE_POSE, LAUNCH_POSE)
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }
}
