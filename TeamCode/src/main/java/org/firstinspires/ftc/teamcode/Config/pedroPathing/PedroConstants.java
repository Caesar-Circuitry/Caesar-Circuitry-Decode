package org.firstinspires.ftc.teamcode.Config.pedroPathing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
          .mass(11.8)
          .headingPIDFCoefficients(new PIDFCoefficients(1.3, 0.0, 0.0, 0.09))//1.5
              .translationalPIDFCoefficients(new PIDFCoefficients(0.065, 0.0, 0.0, 0.02))
              .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.093, 0, 0, 0.9, 0.06))//0.005 0.051
              .useSecondaryHeadingPIDF(true)
              .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                      .5,0.0,0,0.065)) //kf 0.02 kp 2
              .centripetalScaling(0.0005);
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
              .xVelocity(75.78788793368604)
              .yVelocity(62.962014506182335)
              .nominalVoltage(12)
              .useVoltageCompensation(true);


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
