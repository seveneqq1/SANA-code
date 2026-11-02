package org.firstinspires.ftc.teamcode.bobot.controllers;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
public class RGBLight {

    Servo rgb_left,rgb_right;
    private RGBState rgbState = RGBState.RED;
    public ElapsedTime time;
    public enum RGBState {
        GREEN(0.5),
        RED(0.279),
        ;
        final double position;
        RGBState(double position) {
            this.position = position;
        }
    }
    public RGBState getBreakState() {
        return rgbState;
    }

    public RGBLight(HardwareMap hardwareMap) {
      rgb_left =hardwareMap.get(Servo.class,"rgb");
      rgb_right=hardwareMap.get(Servo.class,"rgb2");
      time=new ElapsedTime();
    }
    public void setRGB_PWM_Signal(double position){
        rgb_right.setPosition(position);
        rgb_left.setPosition(position);
    }


    public void update() {
        if(time.milliseconds()>=0 && time.milliseconds()<300){
            rgb_right.setPosition(0.279);
            rgb_left.setPosition(0.611);
        }
        if(time.milliseconds()>300 && time.milliseconds()<600){
            rgb_left.setPosition(0.279);
            rgb_right.setPosition(0.611);
        }
        if(time.milliseconds()>599){
            time.reset();
        }
 
 
 
 
     }

}
