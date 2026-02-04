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
          .forwardZeroPowerAcceleration(-30.87677105808023)
          .lateralZeroPowerAcceleration(-53.42890182464926)
          .translationalPIDFCoefficients(new PIDFCoefficients(.06, 0.0, 0.0, 0.0))
          .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0.0, 0.0, 0.01))
          .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0, 0, 0.051))
          .centripetalScaling(0.005);
  public static MecanumConstants driveConstants =
      new MecanumConstants()
          .maxPower(1)
          .rightFrontMotorName("frontRight")
          .rightRearMotorName("backRight")
          .leftRearMotorName("backLeft")
          .leftFrontMotorName("frontLeft")
          .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
          .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
          .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
          .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
          .xVelocity(73.3870599016487)
          .yVelocity(48.436596487450785);

  public static PinpointConstants localizerConstants =
      new PinpointConstants()
          .hardwareMapName("pinpoint")
          .forwardPodY(-5.5)
          .strafePodX(1.875)
          .distanceUnit(DistanceUnit.INCH)
          .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
          .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
          .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

  public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
        .pathConstraints(pathConstraints)
        .mecanumDrivetrain(driveConstants)
        .pinpointLocalizer(localizerConstants)
        .build();
  }
}
