package org.firstinspires.ftc.teamcode.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.9270045513243188);
        c.yPodOffset.set(-6.58593906192329);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2791996209972422);
                Controller secondaryTranslationalForward = Controller.proportional(0.1031567996072087);
                Controller primaryTranslationalLateral = Controller.proportional(0.36856815513329483);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13617608499926812);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018141847506431125));
                c.brake.set(Controller.proportionalFeedforward(0.015420570380466457));

                c.headingFeedback.set(Controller.proportional(5.056188538951005));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05417419039266479, 0.003785178560080669));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08406689133355406, 0.06753374843010267));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.475923396092864E-4, 9.747064141662521E-4));

                c.maxAchievableForwardVelocity.set(59.01075740726552);
                c.maxAchievableStrafeVelocity.set(49.62090653231916);
                c.naturalForwardDeceleration.set(48.91897538039919);
                c.naturalStrafeDeceleration.set(67.36157342712242);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}