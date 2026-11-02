package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import org.firstinspires.ftc.teamcode.bobot.config.ControlBindings;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.bobot.utilities.VoltageCompensator;

@TeleOp(name = "base", group = "Regional")
public class BaseTeleOp extends OpMode {
    private Mecanum drivetrain;
    private VoltageCompensator voltage;

    @Override
    public void init() {
        drivetrain = new Mecanum(hardwareMap, Constants.driveConfig);
        voltage = new VoltageCompensator(hardwareMap);
        String[] motorNames = {
                Constants.driveConfig.frontLeftName.get(), Constants.driveConfig.backLeftName.get(),
                Constants.driveConfig.frontRightName.get(), Constants.driveConfig.backRightName.get() };
        for (String name : motorNames) {
            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        drivetrain.stop();
    }

    @Override
    public void loop() {
        boolean slow = gamepad1.right_trigger > ControlBindings.TRIGGER_THRESHOLD;
        double scale = (slow ? ControlMath.clamp(RobotConfig.DRIVE_SLOW_SCALE, 0, 1) : 1) * voltage.scale();
        double forward = stick(-gamepad1.left_stick_y) * scale;
        double left = stick(-gamepad1.left_stick_x) * scale;
        double turn = stick(-gamepad1.right_stick_x) * scale;
        drivetrain.drive(new DrivePowers(forward, left, turn), true);

        telemetry.addData("Slow mode", slow);
        telemetry.addData("Battery volts", voltage.getVoltage());
        telemetry.update();
    }

    private static double stick(double value) { return ControlMath.deadband(value, RobotConfig.DRIVE_DEADBAND); }

    @Override
    public void stop() {
        if (drivetrain != null) drivetrain.stop();
    }
}
