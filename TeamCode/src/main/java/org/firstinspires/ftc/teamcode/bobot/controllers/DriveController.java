package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.bobot.utilities.VoltageCompensator;

public class DriveController {
    public enum PoseReset { NONE, CHECKING, CONFIRMED, FAILED }

    private static final double RESET_TOLERANCE_IN = 2.0, RESET_TOLERANCE_DEG = 5.0;
    private static final int RESET_ATTEMPTS = 5;

    private final Localizer localizer;
    private final GoBildaPinpointDriver pinpoint;
    private final Drivetrain drivetrain;
    private final VoltageCompensator voltage;
    private final PillarAvoidance avoidance = new PillarAvoidance();
    private boolean avoiding;
    private Pose requestedPose;
    private int resetAttemptsLeft;
    private PoseReset poseReset = PoseReset.NONE;

    public DriveController(HardwareMap map, VoltageCompensator voltage) {
        localizer = new PinpointLocalizer(map, Constants.localizerConfig);
        pinpoint = map.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        drivetrain = new Mecanum(map, Constants.driveConfig);
        this.voltage = voltage;

        String[] motorNames = {
                Constants.driveConfig.frontLeftName.get(), Constants.driveConfig.backLeftName.get(),
                Constants.driveConfig.frontRightName.get(), Constants.driveConfig.backRightName.get() };
        for (String name : motorNames) {
            DcMotor motor = map.get(DcMotor.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        stop();
    }

    public void updateOdometry() {
        localizer.update();
        if (poseReset == PoseReset.CHECKING) confirmPoseReset();
    }

    private void confirmPoseReset() {
        Pose read = localizer.pose();
        boolean applied = ControlMath.validPose(read)
                && read.distance(requestedPose) <= RESET_TOLERANCE_IN
                && Math.abs(ControlMath.wrapDegrees(Math.toDegrees(read.heading() - requestedPose.heading())))
                        <= RESET_TOLERANCE_DEG;
        if (applied) {
            poseReset = PoseReset.CONFIRMED;
        } else if (--resetAttemptsLeft <= 0) {
            poseReset = PoseReset.FAILED;
        } else {
            localizer.setPose(requestedPose);
        }
    }

    public void drive(double forward, double left, double turn, boolean slowMode) {
        double scale = slowMode ? ControlMath.clamp(RobotConfig.DRIVE_SLOW_SCALE, 0, 1) : 1;
        forward *= scale;
        left *= scale;
        turn *= scale;

        double boost = Math.min(1, voltage.scale());

        Pose pose = localizer.pose();
        if (!ControlMath.validPose(pose)) {
            avoiding = false;
            drivetrain.drive(new DrivePowers(forward * boost, left * boost, turn * boost), true);
            return;
        }

        double heading = pose.heading();
        double fieldX = forward * Math.cos(heading) - left * Math.sin(heading);
        double fieldY = forward * Math.sin(heading) + left * Math.cos(heading);
        PillarAvoidance.Output safe = avoidance.apply(pose, fieldX, fieldY, turn, slowMode);
        avoiding = safe.avoiding;
        drivetrain.drive(toRobotFrame(safe.x * boost, safe.y * boost, safe.turn * boost, heading), true);
    }

    public static DrivePowers toRobotFrame(double x, double y, double turn, double heading) {
        return new DrivePowers(
                x * Math.cos(heading) + y * Math.sin(heading),
                -x * Math.sin(heading) + y * Math.cos(heading),
                turn);
    }

    public Pose getPose() { return localizer.pose(); }

    public void setPose(Pose pose) {
        if (!ControlMath.validPose(pose)) throw new IllegalArgumentException("Invalid odometry pose");
        localizer.setPose(pose);
        requestedPose = pose;
        resetAttemptsLeft = RESET_ATTEMPTS;
        poseReset = PoseReset.CHECKING;
    }

    public PoseReset getPoseReset() { return poseReset; }

    public void addOdometryTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint frequency Hz", pinpoint.getFrequency());
        telemetry.addData("Pinpoint X encoder ticks", pinpoint.getEncoderX());
        telemetry.addData("Pinpoint Y encoder ticks", pinpoint.getEncoderY());
    }

    public boolean isAvoiding() { return avoiding; }

    public void stop() {
        drivetrain.stop();
        avoiding = false;
    }
}
