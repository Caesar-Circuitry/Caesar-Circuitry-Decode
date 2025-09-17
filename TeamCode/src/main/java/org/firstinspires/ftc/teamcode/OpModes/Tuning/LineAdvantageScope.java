package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import static org.firstinspires.ftc.teamcode.Config.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.Config.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.Config.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.Rotation2d;
import org.psilynx.psikit.ftc.PsiKitOpMode;
import org.psilynx.psikit.core.wpi.Pose2d;

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@TeleOp
class LineAdvantageScope extends PsiKitOpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void runOpMode() throws InterruptedException {
        follower.update();
        drawCurrent();
        waitForStart();
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE,0)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE,0), new Pose(0,0)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
        psiKitSetup();
        RLOGServer server = new RLOGServer();
        RLOGWriter writer = new RLOGWriter("logs.rlog");
        server.start();
        writer.start();
        Logger.addDataReceiver(server);
        Logger.addDataReceiver(writer);
        Logger.recordMetadata("some metadata", "string value");
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        Logger.periodicAfterUser(0, 0);
        Logger.periodicBeforeUser();
        processHardwareInputs();
        waitForStart();
        while (opModeIsActive()){
            double beforeUserStart = Logger.getTimestamp();
            Logger.periodicBeforeUser();
            double beforeUserEnd = Logger.getTimestamp();

            processHardwareInputs();
            follower.update();
            drawCurrentAndHistory();

            if (!follower.isBusy()) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            }
            Pose pathPose = follower.getCurrentPath().getClosestPose().getPose();
            Logger.recordOutput("OpMode/path", new Pose2d(pathPose.getX(), pathPose.getY(), new Rotation2d(pathPose.getHeading())));
            Logger.recordOutput("OpMode/pose", new Pose2d(follower.getPose().getX(), follower.getPose().getY(), new Rotation2d(follower.getPose().getHeading())));

            double afterUserStart = Logger.getTimestamp();
            Logger.periodicAfterUser(
                    afterUserStart - beforeUserEnd,
                    beforeUserEnd - beforeUserStart
            );
            if (getPsiKitIsStopRequested()){
                Logger.end();
            }
        }
    }
}

