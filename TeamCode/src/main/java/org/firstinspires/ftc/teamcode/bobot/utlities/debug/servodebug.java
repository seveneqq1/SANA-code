package org.firstinspires.ftc.teamcode.bobot.utlities.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.bobot.utlities.JanaServo;


@TeleOp(name="MOTORDENUG",group = "z")
@Configurable
public class servodebug extends LinearOpMode {


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
        JanaServo servo1= new JanaServo(hardwareMap,"s1");
        JanaServo servo2= new JanaServo(hardwareMap,"s2");
        JanaServo servo3= new JanaServo(hardwareMap,"s3");
        JanaServo servo4= new JanaServo(hardwareMap,"s4");
        JanaServo servo5= new JanaServo(hardwareMap,"s5");
        JanaServo servo6= new JanaServo(hardwareMap,"s6");
         
        waitForStart();



        while (opModeIsActive()) {

            servo1.setPosition(position_servo_1);
            servo2.setPosition(position_servo_2);
            servo3.setPosition(position_servo_3);
            servo4.setPosition(position_servo_4);
            servo5.setPosition(position_servo_5);
            servo6.setPosition(position_servo_6);
            telemetry.update();
             
             
             
             
             
             
             
                    }

    }
}