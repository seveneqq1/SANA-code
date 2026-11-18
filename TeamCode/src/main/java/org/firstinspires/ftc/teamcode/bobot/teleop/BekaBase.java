package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.bobot.Robot;

import java.util.LinkedHashMap;
import java.util.Map;

public class BekaBase extends OpMode {
    protected GamepadEx first_driver, second_driver;
    protected Robot r;
    protected double deltaTime;

    private final ElapsedTime deltaTimer = new ElapsedTime();
    private final Map<LynxModule, LynxModule.BulkCachingMode> previousCacheModes =
            new LinkedHashMap<>();
    private boolean robotInitialized;

    protected void onInit() {}

    protected void onStart() {}

    protected void onUpdate() {}

    @Override
    public final void init() {
        first_driver = new GamepadEx(gamepad1);
        second_driver = new GamepadEx(gamepad2);
        r = Robot.getInstance();
        r.init(hardwareMap);
        robotInitialized = true;
        onInit();
    }

    @Override
    public final void start() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            previousCacheModes.put(hub, hub.getBulkCachingMode());
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        clearBulkCaches();
        deltaTimer.reset();
        onStart();
    }

    @Override
    public final void loop() {
        deltaTime = deltaTimer.seconds();
        deltaTimer.reset();

        clearBulkCaches();
        first_driver.readButtons();
        second_driver.readButtons();
        onUpdate();
        r.periodic();
    }

    @Override
    public final void stop() {
        try {
            if (robotInitialized) {
                clearBulkCaches();
                r.turnOff();
            }
        } finally {
            robotInitialized = false;
            for (Map.Entry<LynxModule, LynxModule.BulkCachingMode> entry
                    : previousCacheModes.entrySet()) {
                entry.getKey().setBulkCachingMode(entry.getValue());
            }
            previousCacheModes.clear();
        }
    }

    private void clearBulkCaches() {
        for (LynxModule hub : previousCacheModes.keySet()) {
            hub.clearBulkCache();
        }
    }
}
