package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.bobot.utlities.JanaServo;

@Configurable
public class TurretController {

    private final JanaServo servo1;
    private final JanaServo servo2;
    public static double center_position=0.505;
    public static double center_target_position=180;
    public boolean forced=true;

    public TurretController(HardwareMap hardwareMap) {
        servo1=new JanaServo(hardwareMap,"turret1");
        servo2=new JanaServo(hardwareMap,"turret2");
        servo2.setSpeedRangePerSecond(2);
        servo1.setSpeedRangePerSecond(2);
    }
    public void start() {
        setTargetPosition(center_target_position);
    }
    double servo_position;
    double ugol;
    public void setTargetPosition(double targetPosition) {
        forced=true;
        if(targetPosition>=25 && targetPosition<=335) {
            if(targetPosition>179.9) {
                ugol = center_position+((targetPosition-180)*0.003222);
            }
            if(targetPosition<179.9){
                ugol=center_position-(-(targetPosition-180)*0.00322222);
            }
            servo_position = ugol;
        }
    }
    public void update() {
        servo2.setPosition(servo_position);
        servo1.setPosition(servo_position);
    }
}