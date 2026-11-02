package org.firstinspires.ftc.teamcode.bobot.utlities.debug;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.bobot.utlities.JanaServo;


@TeleOp(group = "z")
@Configurable
public class Turret_Servos_debug_different_pos extends LinearOpMode {
   public static double servo_position_1 =0.5;
   public static double servo_position_2=0.5;
   public static double servo_speed=2;
   public Motor motor;
    private JanaServo servo3,servo4;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor=new Motor(hardwareMap,"shooter1");
        motor.resetEncoder();
        servo3=new JanaServo(hardwareMap,"turret1");
        servo4=new JanaServo(hardwareMap,"turret2");
        servo3.setSpeedRangePerSecond(servo_speed);
        servo4.setSpeedRangePerSecond(servo_speed);
         
        waitForStart();



        while (opModeIsActive()) {
            servo3.setSpeedRangePerSecond(servo_speed);
            servo4.setSpeedRangePerSecond(servo_speed);
          servo3.setPosition(servo_position_1);
          servo4.setPosition(servo_position_2);
          telemetry.addData("turret current odomtery",(motor.getCurrentPosition()*27/165) /8192*360);
          telemetry.update();
             
             
             
             
             
             
             
             
                    }

    }
}