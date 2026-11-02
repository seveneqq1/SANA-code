package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "beka gey")
public class Beka_gey extends LinearOpMode {
    private Servo turret;
    private Servo jurikta;
    private Servo fl1, fl2;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(Servo.class, "turret");
        fl1 = hardwareMap.get(Servo.class, "fl1");
        jurikta = hardwareMap.get(Servo.class, "turret1");
        fl2 = hardwareMap.get(Servo.class, "fl2");
        waitForStart();
        while(opModeIsActive()){
            turret.setPosition(0.5);
            fl1.setPosition(0.5);
            fl2.setPosition(0.5);
            jurikta.setPosition(0.5);
        }
    }
}
