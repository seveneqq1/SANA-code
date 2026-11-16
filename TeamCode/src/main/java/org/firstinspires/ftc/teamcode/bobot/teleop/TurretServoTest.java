package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Servo Test")
public class TurretServoTest extends LinearOpMode {
    public void runOpMode() {
        Servo s1 = hardwareMap.get(Servo.class, "servo1");
        Servo s2 = hardwareMap.get(Servo.class, "servo2");
        double pos = 0.5;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) pos += 0.003;
            if (gamepad1.dpad_right) pos -= 0.003;
            pos = Math.max(0, Math.min(1, pos));
            boolean only1 = gamepad1.square, only2 = gamepad1.circle;
            if (!only2) s1.setPosition(pos);
            if (!only1) s2.setPosition(pos);
            telemetry.addLine("D-pad left/right = move both");
            telemetry.addLine("Hold Square = only servo1, hold Circle = only servo2");
            telemetry.addData("Position", "%.3f", pos);
            telemetry.addData("servo1 port", s1.getPortNumber());
            telemetry.addData("servo2 port", s2.getPortNumber());
            telemetry.update();
            sleep(10);
        }
    }
}