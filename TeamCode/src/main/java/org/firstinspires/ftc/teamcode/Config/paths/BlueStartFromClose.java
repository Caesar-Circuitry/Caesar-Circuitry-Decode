package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueStartFromClose {
    // Unified pose points with headings (in radians)
    public static final Pose START_POSE = new Pose(21, 122, Math.toRadians(323));
    public static final Pose SLIGHT_MOVE_POSE = new Pose(24, 118.15, Math.toRadians(323));
    public static final Pose LAUNCH_POSE_0 = new Pose(47.772, 93.332, Math.toRadians(200));
    public static final Pose ARTIFACT_0_POSE = new Pose(25.142, 60.386, Math.toRadians(180));
    public static final Pose LAUNCH_POSE = new Pose(57.000, 79.000, Math.toRadians(200));
    public static final Pose LAUNCH_POSE_220 = new Pose(52.000, 79.000, Math.toRadians(220));
    public static final Pose ARTIFACT_1_POSE = new Pose(23.791, 83.807, Math.toRadians(180));
    public static final Pose RAMP_APPROACH_POSE = new Pose(22.000, 69.000, Math.toRadians(180));
    public static final Pose RAMP_INTAKE_POSE = new Pose(9.000, 58.000, Math.toRadians(100));

    // Control points for curves (no heading needed)
    public static final Pose INTAKE_ARTIFACT_0_CONTROL = new Pose(50, 56);
    public static final Pose INTAKE_ARTIFACT_1_CONTROL = new Pose(46.241, 85.758);
    public static final Pose IM_CRASHING_OUT = new Pose(11.54, 64.25);
    public static final Pose RAMP_0_CONTROL = new Pose(51.076, 78.325);
    public static final Pose INTAKE_RAMP_CONTROL = new Pose(18.863, 57.418);

    private final Follower follower;

    public BlueStartFromClose(Follower follower) {
        this.follower = follower;
    }

    public PathChain moveToLaunch0() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, SLIGHT_MOVE_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), SLIGHT_MOVE_POSE.getHeading())
//                .setReversed()
                .build();
    }

    public PathChain moveToClose() {
        return follower.pathBuilder().addPath(
                        new BezierLine(SLIGHT_MOVE_POSE, LAUNCH_POSE_0)
                ).setLinearHeadingInterpolation(SLIGHT_MOVE_POSE.getHeading(), LAUNCH_POSE_0.getHeading())
//                .setReversed()
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
                        new BezierCurve(RAMP_INTAKE_POSE, IM_CRASHING_OUT, LAUNCH_POSE) //Path7
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
                        new BezierCurve(RAMP_INTAKE_POSE, IM_CRASHING_OUT, LAUNCH_POSE) //Path10
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
                        new BezierCurve(RAMP_INTAKE_POSE, IM_CRASHING_OUT, LAUNCH_POSE) //Path13
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }
    public PathChain moveToRamp3() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(LAUNCH_POSE, RAMP_0_CONTROL, RAMP_APPROACH_POSE)
                ).setLinearHeadingInterpolation(LAUNCH_POSE.getHeading(), RAMP_APPROACH_POSE.getHeading())
                .build();
    }

    public PathChain intakeRamp3() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_APPROACH_POSE, INTAKE_RAMP_CONTROL, RAMP_INTAKE_POSE)
                ).setLinearHeadingInterpolation(RAMP_APPROACH_POSE.getHeading(), RAMP_INTAKE_POSE.getHeading())
                .build();
    }

    public PathChain moveToLaunch6() {
        return follower.pathBuilder().addPath(
                        new BezierCurve(RAMP_INTAKE_POSE, IM_CRASHING_OUT, LAUNCH_POSE) //Path7
                ).setLinearHeadingInterpolation(RAMP_INTAKE_POSE.getHeading(), LAUNCH_POSE.getHeading())
                .build();
    }
}
