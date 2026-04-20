package org.firstinspires.ftc.teamcode.Config.pedroPathing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
              .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.08,0.126215598116530128,0.0016782463917316237))
          .forwardZeroPowerAcceleration(-44.287141665772786)
          .lateralZeroPowerAcceleration(-68.97090375570421)
          .headingPIDFCoefficients(new PIDFCoefficients(1, 0.0, 0.0, 0.09))//1.5
              .useSecondaryHeadingPIDF(true)
              .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0.01,0,0.0)) //kf 0.02 kp 2
          .centripetalScaling(0);
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
