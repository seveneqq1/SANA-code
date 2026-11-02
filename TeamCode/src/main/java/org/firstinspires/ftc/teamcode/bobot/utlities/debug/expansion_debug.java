package org.firstinspires.ftc.teamcode.bobot.utlities.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.bobot.utlities.JanaServo;


@TeleOp(group = "z")
@Configurable
public class expansion_debug extends LinearOpMode {



    public static double position_servo_1=0.5;
    public  static double position_servo_2=0.5;
    public  static double position_servo_3=0.5;
    public  static double position_servo_4=0.5;

    public  static double position_servo_5=0.5;
    public  static double position_servo_6=0.5;

    public  static double position_servo_7=0.5;
    public  static double position_servo_8=0.5;
    Motor motor1,motor2,motor3,motor4;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor1=new Motor(hardwareMap,"1");
        motor2=new Motor(hardwareMap,"2");
        motor3=new Motor(hardwareMap,"3");
        motor4=new Motor(hardwareMap,"4");
        JanaServo servo1= new JanaServo(hardwareMap,"s1");
        JanaServo servo2= new JanaServo(hardwareMap,"s2");
        JanaServo servo3= new JanaServo(hardwareMap,"s3");
        JanaServo servo4= new JanaServo(hardwareMap,"s4");
        JanaServo servo5= new JanaServo(hardwareMap,"s5");
        JanaServo servo6= new JanaServo(hardwareMap,"s6");
        JanaServo servo7= new JanaServo(hardwareMap,"s7");
        JanaServo servo8= new JanaServo(hardwareMap,"s8");
         
        waitForStart();


        while (opModeIsActive()) {
            servo1.setPosition(position_servo_1);
            servo2.setPosition(position_servo_2);
            servo3.setPosition(position_servo_3);
            servo4.setPosition(position_servo_4);
            servo5.setPosition(position_servo_5);
            servo6.setPosition(position_servo_6);
            servo7.setPosition(position_servo_7);
            servo8.setPosition(position_servo_8);
            if(gamepad1.y){
                motor1.set(1);
            }
            if(gamepad1.b){
                motor2.set(1);
            }
            if(gamepad1.x){
                motor3.set(1);
            }
            if(gamepad1.a){
                motor4.set(1);
            }
            else{
                motor1.set(0);
                motor2.set(0);
                motor3.set(0);
                motor4.set(0);
            }
            telemetry.addData("1", motor1.getCurrentPosition());
            telemetry.addData("2",motor2.getCurrentPosition());
            telemetry.addData("3", motor3.getCurrentPosition());
            telemetry.addData("4",motor4.getCurrentPosition());
            telemetry.update();
             
             
             
             
             
             
             
             
             
             
             
             
             
             
                    }

    }
}