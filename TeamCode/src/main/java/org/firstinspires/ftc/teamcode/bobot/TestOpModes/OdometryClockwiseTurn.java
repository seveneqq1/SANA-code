package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Odometry - Pod Offset Half Turn", group = "Tuning")
public class OdometryClockwiseTurn extends OpMode {

    private static final double TARGET_DEGREES = 180;
    private static final double TURN_POWER = 0.25;
    private static final double MIN_TURN_POWER = 0.12;
    private static final double TIMEOUT_SECONDS = 30;

    private PinpointLocalizer odometry;
    private Mecanum drive;
    private Pose startPose;
    private double lastHeading, clockwiseDegrees, maxDrift, maxAbsX, maxAbsY;
    private double startedAt;
    private double captureAt = Double.NaN;
    private double savedX = Double.NaN, savedY = Double.NaN, savedDegrees = Double.NaN;
    private boolean turning;
    private String status = "INIT: keep robot still; START runs a clockwise half-turn";

    @Override public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odometry = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        drive = new Mecanum(hardwareMap, Constants.driveConfig);
        String[] names = {
                Constants.driveConfig.frontLeftName.get(), Constants.driveConfig.backLeftName.get(),
                Constants.driveConfig.frontRightName.get(), Constants.driveConfig.backRightName.get()
        };
        for (String name : names) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        drive.stop();
    }

    @Override public void init_loop() {
        odometry.update();
        report(odometry.pose(), 0, 0, 0);
    }

    @Override public void start() {
        odometry.update();
        Pose pose = odometry.pose();
        if (!ControlMath.validPose(pose)) {
            finish("INVALID POSE: restart after checking Pinpoint");
            return;
        }

        startPose = new Pose(pose.x(), pose.y(), pose.heading());
        lastHeading = pose.heading();
        clockwiseDegrees = maxDrift = maxAbsX = maxAbsY = 0;
        captureAt = savedX = savedY = savedDegrees = Double.NaN;
        startedAt = getRuntime();
        turning = true;
        status = "TURNING CLOCKWISE (A aborts)";
    }

    @Override public void loop() {
        odometry.update();
        Pose pose = odometry.pose();
        if (!ControlMath.validPose(pose)) {
            finish("INVALID POSE");
            report(pose, Double.NaN, Double.NaN, 0);
            return;
        }

        double x = 0, y = 0;
        if (startPose != null) {

            clockwiseDegrees -= ControlMath.wrapDegrees(Math.toDegrees(pose.heading() - lastHeading));
            lastHeading = pose.heading();
            double dx = pose.x() - startPose.x(), dy = pose.y() - startPose.y();
            double c = Math.cos(startPose.heading()), s = Math.sin(startPose.heading());
            x = dx * c + dy * s;
            y = -dx * s + dy * c;
            maxAbsX = Math.max(maxAbsX, Math.abs(x));
            maxAbsY = Math.max(maxAbsY, Math.abs(y));
            maxDrift = Math.max(maxDrift, Math.hypot(x, y));
        }

        if (turning) {
            if (gamepad1.a) finish("ABORTED");
            else if (clockwiseDegrees >= TARGET_DEGREES) {
                finish("SETTLING: keep robot still");
                captureAt = getRuntime() + 0.5;
            }
            else if (clockwiseDegrees < -15) finish("WRONG DIRECTION: check drive/heading signs");
            else if (getRuntime() - startedAt >= TIMEOUT_SECONDS) finish("TIMEOUT: check heading updates / turn power");
        }

        double power = 0;
        if (turning) {
            double remaining = TARGET_DEGREES - clockwiseDegrees;
            power = -Math.max(MIN_TURN_POWER, TURN_POWER * Math.min(1, remaining / 60));
            drive.drive(new DrivePowers(0, 0, power), true);
        } else drive.stop();
        if (!Double.isNaN(captureAt) && getRuntime() >= captureAt) {
            savedX = x;
            savedY = y;
            savedDegrees = clockwiseDegrees;
            captureAt = Double.NaN;
            status = "DONE: half-turn readings saved below";
        }
        report(pose, x, y, power);
    }

    private void finish(String reason) {
        turning = false;
        status = reason;
        drive.stop();
    }

    private void report(Pose pose, double x, double y, double power) {
        telemetry.addData("Status", status);
        telemetry.addData("Clockwise turn degrees", clockwiseDegrees);
        telemetry.addData("Turn power", power);
        if (!Double.isNaN(savedDegrees)) {
            telemetry.addData("SAVED turn degrees", savedDegrees);
            telemetry.addData("SAVED forward drift inches", savedX);
            telemetry.addData("SAVED left drift inches", savedY);
        }
        if (ControlMath.validPose(pose)) {
            telemetry.addData("Pose X inches", pose.x());
            telemetry.addData("Pose Y inches", pose.y());
            telemetry.addData("Heading degrees", Math.toDegrees(pose.heading()));
        }
        telemetry.addData("Drift forward inches", x);
        telemetry.addData("Drift left inches", y);
        telemetry.addData("Drift distance inches", Math.hypot(x, y));
        telemetry.addData("Peak forward drift inches", maxAbsX);
        telemetry.addData("Peak left drift inches", maxAbsY);
        telemetry.addData("Peak drift distance inches", maxDrift);
        telemetry.addData("Configured X pod offset inches", Constants.localizerConfig.xPodOffset.get());
        telemetry.addData("Configured Y pod offset inches", Constants.localizerConfig.yPodOffset.get());
        telemetry.addLine("Send the SAVED turn, forward drift, and left drift readings. Robot must pivot in place.");
        telemetry.update();
    }

    @Override public void stop() {
        if (drive != null) drive.stop();
    }
}
