package org.firstinspires.ftc.teamcode.Config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Blue15BallGood {

	/**
	 * ==== PATH CHAINS ====
	 * [CHAIN 1] Intake launch cycle: path1 + path2
	 * [CHAIN 2] Hit gate then intake: path3 + path4
	 * [CHAIN 3] Return to launch: path5
	 * [CHAIN 4] Hit gate then intake: path3 + path4
	 * [CHAIN 5] Return to launch: path5
	 * [CHAIN 6] Park route: path6
	 * Semantic mapping:
	 * moveTo1stLaunch -> path1
	 * moveToIntakeMid -> path2
	 * moveTo2ndLaunch -> path3
	 * moveToIntakeTop -> path4
	 * moveTo3rdLaunch -> path5
	 * moveToGate -> path6
	 */

	// Main waypoints
	public static final Pose START_POSE = new Pose(32,135, Math.toRadians(270));;
	public static final Pose LAUNCH_POSE = new Pose(55.000, 78.000);
	public static final Pose INTAKE_LINEUP = new Pose(18.000, 65.000);
	public static final Pose INTAKE_POINT = new Pose(10.000, 54.000);
	public static final Pose FINAL_POINT = new Pose(61.000, 103.000);

	// Path 2 control points
	public static final Pose P2_C1 = new Pose(50.000, 58.500);
	public static final Pose P2_C2 = new Pose(35.000, 68.000);
	public static final Pose P2_C3 = new Pose(-35.000, 53.000);
	public static final Pose P2_C4 = new Pose(50.000, 58.000);

	// Path 4 control point
	public static final Pose P4_C1 = new Pose(15.000, 58.000);

	// Path 6 control points
	public static final Pose P6_C1 = new Pose(48.000, 94.000);
	public static final Pose P6_C2 = new Pose(35.000, 82.000);
	public static final Pose P6_C3 = new Pose(12.000, 80.000);
	public static final Pose P6_C4 = new Pose(4.000, 89.457);

	private final Follower follower;

	public Blue15BallGood(Follower follower) {
		this.follower = follower;
	}

	public PathChain path1() {
		return follower.pathBuilder().addPath(
						new BezierLine(START_POSE, LAUNCH_POSE)
				).setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(220))
				.build();
	}

	public PathChain path2() {
		return follower.pathBuilder().addPath(
						new BezierCurve(LAUNCH_POSE, P2_C1, P2_C2, P2_C3, P2_C4, INTAKE_LINEUP)
				).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(200))
				.build();
	}

	public PathChain path3() {
		return follower.pathBuilder().addPath(
						new BezierLine(INTAKE_LINEUP, LAUNCH_POSE)
				).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
				.build();
	}

	public PathChain path4() {
		return follower.pathBuilder().addPath(
						new BezierCurve(LAUNCH_POSE, P4_C1, INTAKE_POINT)
				).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
				.build();
	}

	public PathChain path5() {
		return follower.pathBuilder().addPath(
						new BezierLine(INTAKE_POINT, LAUNCH_POSE)
				).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(200))
				.build();
	}

	public PathChain path6() {
		return follower.pathBuilder().addPath(
						new BezierCurve(LAUNCH_POSE, P6_C1, P6_C2, P6_C3, P6_C4, FINAL_POINT)
				).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
				.build();
	}

	// Semantic aliases used by auto for readability.
	public PathChain moveTo1stLaunch() {
		return path1();
	}

	public PathChain moveToIntakeMid() {
		return path2();
	}

	public PathChain moveTo2ndLaunch() {
		return path3();
	}

	public PathChain moveToIntakeTop() {
		return path4();
	}

	public PathChain moveTo3rdLaunch() {
		return path5();
	}

	public PathChain moveToGate() {
		return path6();
	}

	// Explicit chains matching the [CHAIN 1..6] documentation block.
	public PathChain chain1IntakeLaunchCycle() {
		return follower.pathBuilder()
				.addPath(new BezierLine(START_POSE, LAUNCH_POSE))
				.setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(220))
				.addPath(new BezierCurve(LAUNCH_POSE, P2_C1, P2_C2, P2_C3, P2_C4, INTAKE_LINEUP))
				.setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(200))
				.build();
	}

	public PathChain chain2GateThenIntake() {
		return follower.pathBuilder()
				.addPath(new BezierLine(INTAKE_LINEUP, LAUNCH_POSE))
				.setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
				.addPath(new BezierCurve(LAUNCH_POSE, P4_C1, INTAKE_POINT))
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
				.build();
	}

	public PathChain chain3ReturnToLaunch() {
		return follower.pathBuilder()
				.addPath(new BezierLine(INTAKE_POINT, LAUNCH_POSE))
				.setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(200))
				.build();
	}

	public PathChain chain4GateThenIntake() {
		return chain2GateThenIntake();
	}

	public PathChain chain5ReturnToLaunch() {
		return chain3ReturnToLaunch();
	}

	public PathChain chain6ParkRoute() {
		return follower.pathBuilder()
				.addPath(new BezierCurve(LAUNCH_POSE, P6_C1, P6_C2, P6_C3, P6_C4, FINAL_POINT))
				.setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
				.build();
	}

	// Chain travel-only segments so predictive braking does not force a stop mid-cycle.
	public PathChain intakeMidTo2ndLaunchChain() {
		return follower.pathBuilder()
				.addPath(new BezierCurve(LAUNCH_POSE, P2_C1, P2_C2, P2_C3, P2_C4, INTAKE_LINEUP))
				.setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(200))
				.addPath(new BezierLine(INTAKE_LINEUP, LAUNCH_POSE))
				.setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(180))
				.build();
	}

	public PathChain intakeTopTo3rdLaunchChain() {
		return follower.pathBuilder()
				.addPath(new BezierCurve(LAUNCH_POSE, P4_C1, INTAKE_POINT))
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
				.addPath(new BezierLine(INTAKE_POINT, LAUNCH_POSE))
				.setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(200))
				.build();
	}
}
