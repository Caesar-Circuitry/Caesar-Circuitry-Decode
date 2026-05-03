package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class WorldsQ11 {
    public static final Pose START_POSE_Q11 = new Pose(56,8, Math.toRadians(180));
    public static final Pose END_POSE_Q11 = new Pose(8,8,Math.toRadians(180));
    public static final Pose GRAH = new Pose(57,8,Math.toRadians(180));

    private Follower follower;
    public WorldsQ11(Follower follower) {
        this.follower = follower;
    }

    public PathChain moveTo1stLaunch() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE_Q11, END_POSE_Q11)
                ).setLinearHeadingInterpolation(START_POSE_Q11.getHeading(), END_POSE_Q11.getHeading())
                .build();
    }
    public PathChain movething() {
        return follower.pathBuilder().addPath(
                new BezierLine(END_POSE_Q11,GRAH)
        ).setLinearHeadingInterpolation(END_POSE_Q11.getHeading(),GRAH.getHeading())
                .build();
    }


}
