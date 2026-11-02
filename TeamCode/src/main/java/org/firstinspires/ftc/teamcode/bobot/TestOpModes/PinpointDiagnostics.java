package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Pinpoint - Raw Pod Check", group = "Tuning")
public class PinpointDiagnostics extends OpMode {
    private PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private Pose startPose;
    private int startTicksX, startTicksY;

    @Override public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    }

    @Override public void init_loop() {
        localizer.update();
        report();
    }

    @Override public void start() {
        localizer.update();
        Pose pose = localizer.pose();
        startPose = new Pose(pose.x(), pose.y(), pose.heading());
        startTicksX = pinpoint.getEncoderX();
        startTicksY = pinpoint.getEncoderY();
    }

    @Override public void loop() {
        localizer.update();
        report();
    }

    private void report() {
        Pose pose = localizer.pose();
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint frequency Hz", pinpoint.getFrequency());
        telemetry.addData("Raw X encoder ticks", pinpoint.getEncoderX());
        telemetry.addData("Raw Y encoder ticks", pinpoint.getEncoderY());
        telemetry.addData("Device X inches", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Device Y inches", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Device heading degrees", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pedro pose X inches", pose.x());
        telemetry.addData("Pedro pose Y inches", pose.y());
        telemetry.addData("Configured pod type", Constants.localizerConfig.podType.get());
        telemetry.addData("Custom encoder resolution", Constants.localizerConfig.ticksPerUnit.get());
        if (startPose != null) {
            double dx = pose.x() - startPose.x(), dy = pose.y() - startPose.y();
            double c = Math.cos(startPose.heading()), s = Math.sin(startPose.heading());
            telemetry.addData("Forward travel inches", c * dx + s * dy);
            telemetry.addData("Left travel inches", -s * dx + c * dy);
            telemetry.addData("X ticks since START", pinpoint.getEncoderX() - startTicksX);
            telemetry.addData("Y ticks since START", pinpoint.getEncoderY() - startTicksY);
        }
        telemetry.addLine("Keep still during INIT. START, then push forward 12 inches without rotating.");
        telemetry.addLine("Repeat moving left. For the configured 4-bar pods, expect about 505 ticks per inch.");
        telemetry.update();
    }
}
