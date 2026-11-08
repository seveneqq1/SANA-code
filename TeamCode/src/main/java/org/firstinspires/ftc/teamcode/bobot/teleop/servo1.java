package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servo feed")
public class servo1 extends LinearOpMode {
    Servo servo;
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class, "stopper");

        waitForStart();

        while(opModeIsActive()){
            servo.setPosition(0.5);
        }
    }
}
