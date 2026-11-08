package org.firstinspires.ftc.teamcode.bobot.controller;

/**
 * Turret aimed by two standard servos (servo1 and servo2) that move together.
 * 180 degrees is center. Convert degrees to servo position with kV.
 */

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class turretController {
    private final Servo servo1;
    private final Servo servo2;
    public static double center_pos = 0.5;
    public static double center_target_pos = 180;
    public static double kV = 1.0 / 422.0;

    public static double min_target_pos = -60;
    public static double max_target_pos = 420;

    public boolean forced = true;

    public turretController(HardwareMap hardwareMap){
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    public void start(){
        setTargetPos(center_target_pos);
    }

    double servo_pos = center_pos; // start centered so the first update() does not slam to 0
    double angle;

    public void setTargetPos(double targetPos){
        forced = true;
        if(targetPos < min_target_pos || targetPos > max_target_pos) return;
        double pos = center_pos + (targetPos - center_target_pos) * kV;
        if(pos < 0 || pos > 1) return;
        servo_pos = pos;
    }
    public void update(){
        servo1.setPosition(servo_pos);
        servo2.setPosition(servo_pos);
    }
}
