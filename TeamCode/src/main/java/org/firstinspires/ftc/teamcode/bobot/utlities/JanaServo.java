package org.firstinspires.ftc.teamcode.bobot.utlities;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class JanaServo {
    Servo servo;
    public double speedRangePerSecond = 0.85;
    public double estimatedPosition = 0.5;
    public double targetPosition = 0.5;

    ElapsedTime actionTimer = new ElapsedTime();
    public JanaServo(HardwareMap hardwareMap, String name){
        servo = hardwareMap.servo.get(name);
    }
    public void setSpeedRangePerSecond(double speed){
        speedRangePerSecond = speed;
    }
    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }
    public void setPosition(double position){
            updateEstimatedPos();
            if (position < 0.0 || 1.0 < position)
                return;
            servo.setPosition(position);
            targetPosition = position;
    }
    public void updateEstimatedPos(){
        double deltaSeconds = actionTimer.milliseconds() / 1000.0;
        actionTimer.reset();
        double errorMagnitude = Math.abs(targetPosition - estimatedPosition);
        if (errorMagnitude == 0)
            return;
        double direction = (targetPosition - estimatedPosition) / errorMagnitude;
        double voltageCompensationMultiplier = 1.09;
 
        double increase = Math.min(errorMagnitude, speedRangePerSecond * voltageCompensationMultiplier * deltaSeconds) * direction;
        estimatedPosition += increase;
    }
    public boolean getBusy(){
        updateEstimatedPos();
        return estimatedPosition != targetPosition;
    }

    public void pwmDisable(){
        servo.getController().pwmDisable();
    }
    public void pwmEnable(){servo.getController().pwmEnable();}
    public void getPwmStatus(ServoController.PwmStatus pwmStatus){servo.getController().getPwmStatus();}
}
