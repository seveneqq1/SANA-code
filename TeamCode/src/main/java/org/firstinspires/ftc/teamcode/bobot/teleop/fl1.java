package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "servo1fl")
public class fl1 extends OpMode {
    Servo fl;
    public void init(){
        fl = hardwareMap.get(Servo.class, "fl");
    }
    public void loop(){
        double power = gamepad1.left_stick_y;
        fl.setPosition(power);
        telemetry.addData("flPos", fl.getPosition());
        telemetry.update();
    }
}
