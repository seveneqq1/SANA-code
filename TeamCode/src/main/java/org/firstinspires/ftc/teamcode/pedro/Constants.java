package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.559);
        c.yPodOffset.set(-4.789);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftf");
        c.backLeftName.set("leftb");
        c.frontRightName.set("rightfr");
        c.backRightName.set("rightb");

        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        c.manualBrakeMode.set(true);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2775291954617127);
                Controller secondaryTranslationalForward = Controller.proportional(0.10253962200642289);
                Controller primaryTranslationalLateral = Controller.proportional(0.3819550289260989);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14112217716185357);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009115022531997254));
                c.brake.set(Controller.proportionalFeedforward(0.007747769152197666));

                c.headingFeedback.set(Controller.proportional(4.246281437144492));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.056499913313723324, 0.0074430706233957105));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1852025201307832, 0.041875019577302094));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.8355285081160156E-4, 0.0025723217801619284));

                c.maxAchievableForwardVelocity.set(95.70640091776855);
                c.maxAchievableStrafeVelocity.set(78.51523429988089);
                c.naturalForwardDeceleration.set(25.188809415408286);
                c.naturalStrafeDeceleration.set(55.123108153526104);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
}
