package org.firstinspires.ftc.teamcode.bobot.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class Flower extends SubsystemBase {
    private Servo flower;
    private boolean beka = false;
    public static double OPEN_POS = 0.5, CLOSE_POS = 0;
    public void init(HardwareMap hw){
        flower = hardwareMap.get(Servo.class, "flower");
    }
    public void setPos(double pos){
        flower.setPosition(pos);
    }
    public void openFlower(){
        beka = true;
        setPos(OPEN_POS);
    }
    public void closeFlower(){
        beka = false;
        setPos(CLOSE_POS);
    }
    public boolean isFlowerOpen(){ return beka; }
}
