package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;

public class FlowerTakerController {
    private final Servo servo1, servo2;
    private boolean open;

    public FlowerTakerController(HardwareMap map) {
        this(map.get(Servo.class, RobotConfig.FLOWER_TAKER_1_NAME), map.get(Servo.class, RobotConfig.FLOWER_TAKER_2_NAME));
    }

    public FlowerTakerController(Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public void update() {
        servo1.setPosition(open ? RobotConfig.FLOWER_TAKER_1_OPEN_POSITION : RobotConfig.FLOWER_TAKER_1_CLOSED_POSITION);
        servo2.setPosition(open ? RobotConfig.FLOWER_TAKER_2_OPEN_POSITION : RobotConfig.FLOWER_TAKER_2_CLOSED_POSITION);
    }

    public void toggle() { open = !open; }

    public void open() { open = true; }

    public void close() { open = false; }

    public boolean isOpen() { return open; }
}
