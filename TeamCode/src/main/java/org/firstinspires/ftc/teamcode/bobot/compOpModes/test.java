package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test")
public class test extends LinearOpMode {
    private DcMotor motor;
    private Servo servo;
    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a) {
                motor.setPower(1);
            } else {
                motor.setPower(0);
            }
        }

    }

}
