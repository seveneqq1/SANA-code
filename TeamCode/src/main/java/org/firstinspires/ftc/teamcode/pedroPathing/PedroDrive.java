package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Objects;
import java.util.OptionalDouble;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * The sole owner of the four drive motors and Pinpoint in an OpMode that follows paths.
 * Call update once per active loop and stop during OpMode cleanup after cancelling commands.
 */
public final class PedroDrive {
    private final Follower follower;

    /** Initialize while stationary with a Foresight config measured on this robot in inches. */
    public PedroDrive(HardwareMap hardwareMap, ForesightConfig tunedConfig) {
        this(createFollower(hardwareMap, tunedConfig));
    }

    private PedroDrive(Follower follower) {
        this.follower = follower;
    }

    private static Follower createFollower(HardwareMap hardwareMap, ForesightConfig tunedConfig) {
        Foresight foresight = new Foresight(Objects.requireNonNull(tunedConfig, "tunedConfig"));
        PinpointConfig pinpoint = new PinpointConfig(config -> {
            config.name.set(Constants.PINPOINT_NAME);
            config.xPodOffset.set(Constants.PointTrack.X_POD_OFFSET_MM);
            config.yPodOffset.set(Constants.PointTrack.Y_POD_OFFSET_MM);
            config.offsetUnits.set(DistanceUnit.MM);
            config.ticksPerUnit.set(OptionalDouble.of(Constants.PointTrack.TICKS_PER_MM));
            config.encoderResolutionUnit.set(DistanceUnit.MM);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
            config.xPodDirection.set(Constants.PointTrack.REVERSE_X_POD
                    ? EncoderDirection.REVERSED : EncoderDirection.FORWARD);
            config.yPodDirection.set(Constants.PointTrack.REVERSE_Y_POD
                    ? EncoderDirection.REVERSED : EncoderDirection.FORWARD);
            config.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        });
        MecanumConfig motors = new MecanumConfig(config -> {
            config.frontLeftName.set(Constants.FRONT_LEFT);
            config.frontRightName.set(Constants.FRONT_RIGHT);
            config.backLeftName.set(Constants.BACK_LEFT);
            config.backRightName.set(Constants.BACK_RIGHT);
            config.frontLeftDirection.set(Direction.FORWARD);
            config.frontRightDirection.set(Direction.REVERSE);
            config.backLeftDirection.set(Direction.FORWARD);
            config.backRightDirection.set(Direction.REVERSE);
        });
        return new Follower(new PinpointLocalizer(hardwareMap, pinpoint),
                new Mecanum(hardwareMap, motors), foresight);
    }

    /** Call once per active loop, before the scheduler checks the command's completion. */
    public void update() {
        follower.update();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose pose() {
        return follower.pose();
    }

    /** Follow a path through completion, then leave the normal Pedro end hold active. */
    public Command follow(Path path) {
        return PedroCommands.follow(follower, path)
                .setDone(() -> !follower.following())
                .requiring(this)
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) stop();
                });
    }

    /** Hold until interrupted or cancelled. Use as the last step of a sequence. */
    public Command hold(Pose pose) {
        return PedroCommands.hold(follower, pose)
                .setDone(() -> false)
                .requiring(this)
                .setEnd(condition -> stop());
    }

    /** Zero motor power immediately, including an end hold after a completed command. */
    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
    }
}
