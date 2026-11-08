package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "turret set 0.5 both")
public class servo2 extends LinearOpMode {
    Servo servo;
    Servo servo1;
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class, "servo2");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(0.5);
            servo1.setPosition(0.5);
        }
    }
}
