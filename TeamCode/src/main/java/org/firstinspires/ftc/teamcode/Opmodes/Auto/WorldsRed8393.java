package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class WorldsRed8393 {
    public static final Pose START_POSE = new Pose(90,8,Math.toRadians(270));
    public static final Pose END_POSE = new Pose(135,8,Math.toRadians(270));

    private Follower follower;
    public WorldsRed8393(Follower follower) {
        this.follower = follower;
    }

    public PathChain red() {
        return follower.pathBuilder().addPath(
                        new BezierLine(START_POSE, END_POSE)
                ).setLinearHeadingInterpolation(START_POSE.getHeading(), END_POSE.getHeading())
                .build();
    }
}
