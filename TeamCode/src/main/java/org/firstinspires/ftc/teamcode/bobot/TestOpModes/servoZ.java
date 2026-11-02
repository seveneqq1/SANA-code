package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoZ")
public class servoZ extends LinearOpMode {
    private Servo zeee;

    @Override
    public void runOpMode() throws InterruptedException {
        zeee = hardwareMap.get(Servo.class, "veee");

        waitForStart();

        while (opModeIsActive()) {
            zeee.setPosition(0.5);
            telemetry.addData("Servo Position", zeee.getPosition());
            telemetry.update();
        }
    }
}
