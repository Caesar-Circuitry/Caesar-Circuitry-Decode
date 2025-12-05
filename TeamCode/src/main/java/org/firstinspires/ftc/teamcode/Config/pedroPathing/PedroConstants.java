package org.firstinspires.ftc.teamcode.Config.pedroPathing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroConstants {
  public static FollowerConstants followerConstants =
      new FollowerConstants()
          .mass(8.8)
          .forwardZeroPowerAcceleration(-30.4571698665302)
          .lateralZeroPowerAcceleration(-65.82404523256889)
          .translationalPIDFCoefficients(new PIDFCoefficients(.06, 0.0, 0.0, 0.0))
          .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.0, 0.0, 0.01))
          .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0, 0, 0.051))
          .centripetalScaling(0.005);
  public static MecanumConstants driveConstants =
      new MecanumConstants()
          .maxPower(1)
          .rightFrontMotorName("FRM")
          .rightRearMotorName("BRM")
          .leftRearMotorName("BLM")
          .leftFrontMotorName("FLM")
          .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
          .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
          .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
          .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
          .xVelocity(78.88153412586122)
          .yVelocity(63.368909940944874);

  public static PinpointConstants localizerConstants =
      new PinpointConstants()
          .forwardPodY(7 / 16)
          .strafePodX(3)
          .distanceUnit(DistanceUnit.INCH)
          .hardwareMapName("pinpoint")
          .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
          .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
          .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pathConstraints(pathConstraints)
        .mecanumDrivetrain(driveConstants)
        .pinpointLocalizer(localizerConstants)
        .build();
  }
}
