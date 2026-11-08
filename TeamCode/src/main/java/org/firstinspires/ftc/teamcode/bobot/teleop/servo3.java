package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "shoot seq 0.7 +intake+trans")
public class servo3 extends LinearOpMode {
    DcMotor shooter1;
    DcMotor shooter2;
    DcMotor intake, transfer;
    Servo stopper;
    boolean intake_on = false;
    public void runOpMode(){
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "trans");
        stopper = hardwareMap.get(Servo.class, "stopper");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                intake_on = !intake_on;
            }
            if(intake_on){
                intake.setPower(1);
                transfer.setPower(-1);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }
            if(gamepad1.x){
                stopper.setPosition(0.5);
            } else{
                stopper.setPosition(0.05);
            }
            shooter1.setPower(0.5);
            shooter2.setPower(-0.5);
        }
    }
}
