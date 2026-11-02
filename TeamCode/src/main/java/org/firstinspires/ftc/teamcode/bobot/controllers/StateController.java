package org.firstinspires.ftc.teamcode.bobot.controllers;

import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;
import org.firstinspires.ftc.teamcode.bobot.utilities.VoltageCompensator;
import org.firstinspires.ftc.teamcode.bobot.utilities.TurretAim;

public class StateController {
    public enum Mode { IDLE, INTAKING, EJECTING, SPINNING_UP, SHOOTING }

    public final VoltageCompensator voltage;
    public final ShootingSystem shooting;
    public final TurretController turret;

    private Mode mode = Mode.IDLE;
    private boolean turretAlwaysOn;

    public StateController(HardwareMap map, Alliance alliance) {
        this(map, new VoltageCompensator(map), alliance);
    }

    private StateController(HardwareMap map, VoltageCompensator voltage, Alliance alliance) {
        this(voltage, new ShootingSystem(map), new TurretController(map, alliance));
        for (LynxModule hub : map.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public StateController(VoltageCompensator voltage, ShootingSystem shooting, TurretController turret) {
        this.voltage = voltage;
        this.shooting = shooting;
        this.turret = turret;
    }

    public void update(Pose pose) {
        boolean shootRequested = mode == Mode.SHOOTING;
        boolean spinningUp = mode == Mode.SPINNING_UP;

        if (turretAlwaysOn || spinningUp || shootRequested) turret.aim();
        else turret.hold();
        turret.update(pose);

        switch (mode) {
            case SHOOTING: shooting.update(ShootingSystem.Mode.SHOOT); break;
            case SPINNING_UP: shooting.update(ShootingSystem.Mode.SPINUP); break;
            case INTAKING: shooting.update(ShootingSystem.Mode.COLLECT); break;
            case EJECTING: shooting.update(ShootingSystem.Mode.EJECT); break;
            default: shooting.stop();
        }
    }

    public void intake() { mode = Mode.INTAKING; }
    public void eject() { mode = Mode.EJECTING; }
    public void shoot() { mode = Mode.SHOOTING; }
    public void spinUp() { mode = Mode.SPINNING_UP; }
    public void idle() { mode = Mode.IDLE; }

    public void setMode(Mode mode) { this.mode = mode; }

    public void setTurretAlwaysOn(boolean on) { turretAlwaysOn = on; }

    public Mode getMode() { return mode; }

    public boolean isReadyToShoot() { return shooting.isReady(); }

    public boolean isFeeding() { return shooting.isFeeding(); }

    public void stop() {
        mode = Mode.IDLE;
        turretAlwaysOn = false;
        shooting.stop();
        turret.hold();
    }

    public void reset() {
        stop();
        turret.reset();
    }

    public void addTelemetry(Telemetry telemetry, Pose pose) {
        telemetry.addData("Mode", mode);
        telemetry.addData("Battery volts", voltage.getVoltage());
        telemetry.addData("Pose", pose);
        telemetry.addData("Pose X inches", pose.x());
        telemetry.addData("Pose Y inches", pose.y());
        telemetry.addData("Pose heading degrees", Math.toDegrees(pose.heading()));
        telemetry.addData("Shot distance inches", turret.getTargetDistanceInches());
        shooting.addTelemetry(telemetry);
        telemetry.addData("Aim target (split Y)", turret.getHiveHalf());
        telemetry.addData("Turret pivot Y inches", TurretAim.pivot(pose).y());
        telemetry.addData("Half switch center Y inches", RobotConfig.TARGET_SPLIT_Y);
        double halfHysteresis = Math.max(0, RobotConfig.TARGET_SWITCH_HYSTERESIS_IN);
        telemetry.addData("Switch to UPPER above Y", RobotConfig.TARGET_SPLIT_Y + halfHysteresis);
        telemetry.addData("Switch to LOWER at/below Y", RobotConfig.TARGET_SPLIT_Y - halfHysteresis);
        telemetry.addData("Moving shot lead seconds", RobotConfig.SHOOT_FLIGHT_TIME_SECONDS);
        telemetry.addData("Target X inches", turret.getTargetX());
        telemetry.addData("Target Y inches", turret.getTargetY());
        telemetry.addData("Target field bearing degrees", turret.getFieldBearingDegrees());
        telemetry.addData("Turret requested degrees from backward", turret.getRequestedAngleDegrees());
        telemetry.addData("Turret servo command (both)", TurretController.FIXED_POSITION);
        telemetry.addData("Turret target degrees", turret.getTargetAngleDegrees());
        telemetry.addData("Turret estimated degrees", turret.getEstimatedAngleDegrees());
        telemetry.addData("Turret trim degrees", turret.getTrimDegrees());
        telemetry.addData("Turret reachable / aimed", "%s / %s", turret.isTargetReachable(), turret.isAimed());
    }
}
