package org.firstinspires.ftc.teamcode.bobot.utlities.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.bobot.utlities.JanaServo;


@TeleOp(name="MOTORDENUG",group = "z")
@Configurable
public class motorsdebug extends LinearOpMode {


    private Motor motor2;
    public static double position_servo_1=0.5;
    public  static double position_servo_2=0.5;
    public  static double position_servo_3=0.5;
    public  static double position_servo_4=0.5;
    public  static double position_servo_5=0.5;
    public  static double position_servo_6=0.5;
    private Motor motor1;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor2 =new Motor(hardwareMap,"2");
        motor1 =new Motor(hardwareMap,"1");
        Motor motor3=new Motor(hardwareMap,"3");
        Motor motor4=new Motor(hardwareMap,"4");
        Motor motor5=new Motor(hardwareMap,"5");
        Motor motor6=new Motor(hardwareMap,"6");
        Motor motor7=new Motor(hardwareMap,"7");
        Motor motor8=new Motor(hardwareMap,"8");
        JanaServo servo1= new JanaServo(hardwareMap,"s1");
        JanaServo servo2= new JanaServo(hardwareMap,"s2");
        JanaServo servo3= new JanaServo(hardwareMap,"s3");
        JanaServo servo4= new JanaServo(hardwareMap,"s4");
        JanaServo servo5= new JanaServo(hardwareMap,"s5");
        JanaServo servo6= new JanaServo(hardwareMap,"s6");
        motor2.setInverted(true);
         
        waitForStart();

        motor1.resetEncoder();
        motor2.resetEncoder();
        motor3.resetEncoder();
        motor4.resetEncoder();
        motor5.resetEncoder();
        motor6.resetEncoder();
        motor7.resetEncoder();
        motor8.resetEncoder();

        while (opModeIsActive()) {
            if(gamepad1.a){
              motor3.set(1);
            }
            else if(gamepad1.b){
                motor1.set(1);
            }
            else if(gamepad1.y){
                motor2.set(1);
            }
            else if(gamepad1.x){
                motor4.set(1);
            }
            else if(gamepad1.dpad_up){
                motor5.set(1);
            }
            else if(gamepad1.dpad_down){
                motor6.set(1);
            }
            else if(gamepad1.dpad_left){
                motor7.set(1);
            }
            else if(gamepad1.dpad_right){
                motor8.set(1);
            }

            else {
                motor1.set(0);
                motor2.set(0);
                motor3.set(0);
                motor4.set(0);
                motor5.set(0);
                motor6.set(0);
                motor7.set(0);
                motor8.set(0);
            }
            servo1.setPosition(position_servo_1);
            servo2.setPosition(position_servo_2);
            servo3.setPosition(position_servo_3);
            servo4.setPosition(position_servo_4);
            servo5.setPosition(position_servo_5);
            servo6.setPosition(position_servo_6);
            telemetry.addData("1", motor1.getCurrentPosition());
            telemetry.addData("2", motor2.getCurrentPosition());
            telemetry.addData("3",motor3.getCurrentPosition());
            telemetry.addData("4",motor4.getCurrentPosition());
            telemetry.addData("5",motor5.getCurrentPosition());
            telemetry.addData("6",motor6.getCurrentPosition());
            telemetry.addData("7",motor7.getCurrentPosition());
            telemetry.addData("8",motor8.getCurrentPosition());
            telemetry.update();
            
             
             
             
             
             
             
             
             
             
             
             
             
                    }

    }
}