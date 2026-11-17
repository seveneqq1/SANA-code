package org.firstinspires.ftc.teamcode.bobot.controllers;

/**
 * Turret aimed by two standard servos (servo1 and servo2) that move together.
 * 180 degrees is center. Convert degrees to servo position with kV.
 */

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class TurretController {
    private final Servo servo1;
    private final Servo servo2;
    public static double center_pos = 0.062; // servo position where the turret looks straight forward (measured Nov 14)
    public static double center_target_pos = 180;
    public static double kV = 1.0 / 422.0; // 330 deg servo * 80/20 * 44/137 = ~424 deg; measured ~0.217 per 90 deg

    public static double min_target_pos = -60;
    public static double max_target_pos = 600;

    // The servos are mounted mirrored, so one of them must be reversed to move the same way.
    // If they fight each other, flip ONLY ONE value (flipping both keeps them fighting).
    // Editable live in Panels.
    public static boolean REVERSE_SERVO1 = false;
    public static boolean REVERSE_SERVO2 = false;

    private boolean appliedReverse1, appliedReverse2;

    public TurretController(HardwareMap hardwareMap){
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        applyDirections();
    }

    public void start(){
        setTargetPos(center_target_pos);
    }

    private double servoPosition = center_pos;

    public void setTargetPos(double targetPos){
        if(targetPos < min_target_pos || targetPos > max_target_pos) return;
        double pos = center_pos + (targetPos - center_target_pos) * kV;
        if(pos < 0 || pos > 1) return;
        servoPosition = pos;
    }
    public void update(){
        if (REVERSE_SERVO1 != appliedReverse1 || REVERSE_SERVO2 != appliedReverse2) {
            applyDirections();
        }
        // Both get the same position; the direction flag mirrors one of them.
        servo1.setPosition(servoPosition);
        servo2.setPosition(servoPosition);
    }

    private void applyDirections() {
        servo1.setDirection(REVERSE_SERVO1 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo2.setDirection(REVERSE_SERVO2 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        appliedReverse1 = REVERSE_SERVO1;
        appliedReverse2 = REVERSE_SERVO2;
    }
}
