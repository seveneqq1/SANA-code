package org.firstinspires.ftc.teamcode.bobot.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.bobot.states.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "Timed Forward Auto", group = "Regional")
public class TimedForwardAuto extends LinearOpMode {

    public static double DRIVE_SECONDS = 5;
    public static double DRIVE_POWER = 0.2;

    private final DcMotor[] motors = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            PoseStorage.clear();
            motors[0] = hardwareMap.get(DcMotor.class, Constants.driveConfig.frontLeftName.get());
            motors[1] = hardwareMap.get(DcMotor.class, Constants.driveConfig.backLeftName.get());
            motors[2] = hardwareMap.get(DcMotor.class, Constants.driveConfig.frontRightName.get());
            motors[3] = hardwareMap.get(DcMotor.class, Constants.driveConfig.backRightName.get());
            DcMotorSimple.Direction[] directions = {
                    Constants.driveConfig.frontLeftDirection.get(),
                    Constants.driveConfig.backLeftDirection.get(),
                    Constants.driveConfig.frontRightDirection.get(),
                    Constants.driveConfig.backRightDirection.get()
            };
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(0);
                motors[i].setDirection(directions[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            telemetry.addLine("Ready: timed forward drive, no odometry.");
            telemetry.addData("Drive seconds", DRIVE_SECONDS);
            telemetry.addData("Drive power (0..1)", DRIVE_POWER);
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            final double seconds = DRIVE_SECONDS;
            final double power = DRIVE_POWER;
            if (!Double.isFinite(seconds) || seconds < 0
                    || !Double.isFinite(power) || power < 0 || power > 1) {
                telemetry.addLine("Invalid settings: seconds must be >= 0, power must be 0..1 (both finite).");
                telemetry.update();
                return;
            }

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (opModeIsActive() && timer.seconds() < seconds) {
                setDrivePower(power);
                telemetry.addData("Remaining seconds", "%.2f", Math.max(0, seconds - timer.seconds()));
                telemetry.addData("Drive power", power);
                telemetry.update();
                idle();
            }
        } finally {
            setDrivePower(0);
        }
    }

    private void setDrivePower(double power) {
        for (DcMotor motor : motors) {
            if (motor != null) motor.setPower(power);
        }
    }
}
