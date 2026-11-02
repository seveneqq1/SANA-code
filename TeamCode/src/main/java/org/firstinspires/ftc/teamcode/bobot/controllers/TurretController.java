package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.bobot.utilities.TurretAim;

public class TurretController {

    public static final double FIXED_POSITION = 0.5;

    private final Servo servo, secondServo;
    private final Alliance alliance;
    private boolean tracking;
    private TurretAim.HiveHalf half;
    private double heldAngle, trim, targetAngle, estimatedAngle;
    private boolean reachable, validAim;
    private long lastUpdate;
    private Pose previousPivot;
    private double targetX = Double.NaN, targetY = Double.NaN;
    private double fieldBearing = Double.NaN, requestedAngle = Double.NaN;
    private double targetDistance = Double.NaN;
    private Double manualAngle;
    public TurretController(HardwareMap map, Alliance alliance) {
        this(map.get(Servo.class, RobotConfig.TURRET_NAME),
                map.get(Servo.class, RobotConfig.TURRET_SECOND_NAME), alliance);
    }
    public TurretController(Servo servo, Servo secondServo, Alliance alliance) {
        this.servo = servo;
        this.secondServo = secondServo;
        this.alliance = alliance;
        servo.setDirection(RobotConfig.TURRET_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        secondServo.setDirection(RobotConfig.TURRET_SECOND_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        holdFixedPosition();
    }
    public void update(Pose robotPose) {
        long now = System.nanoTime();
        double dt = lastUpdate == 0 ? 0.02 : (now - lastUpdate) / 1e9;
        lastUpdate = now;
        update(robotPose, dt);
    }

    public void update(Pose robotPose, double dt) {
        Pose pivot = ControlMath.validPose(robotPose) ? TurretAim.pivot(robotPose) : null;
        double velocityX = 0, velocityY = 0;
        if (pivot != null && previousPivot != null && dt > 0 && dt <= .1) {
            double x = (pivot.x() - previousPivot.x()) / dt;
            double y = (pivot.y() - previousPivot.y()) / dt;
            if (Math.hypot(x, y) <= RobotConfig.SHOOT_MAX_TRACKING_SPEED_IPS) {
                velocityX = x;
                velocityY = y;
            }
        }
        previousPivot = pivot;

        validAim = false;
        reachable = false;
        targetDistance = targetX = targetY = fieldBearing = requestedAngle = Double.NaN;
        if (manualAngle != null) targetAngle = clampToLimits(manualAngle);
        else if (!tracking) targetAngle = clampToLimits(heldAngle);

        if (pivot != null) {

            half = TurretAim.select(pivot, half);
            Pose target = TurretAim.hive(alliance, half);
            target = TurretAim.movingShotTarget(target, velocityX, velocityY, RobotConfig.SHOOT_FLIGHT_TIME_SECONDS);
            targetX = target.x();
            targetY = target.y();
            targetDistance = pivot.distance(target);

            fieldBearing = Math.toDegrees(Math.atan2(targetY - pivot.y(), targetX - pivot.x()));
            double robotAngle = fieldBearing - Math.toDegrees(pivot.heading())
                    + RobotConfig.TURRET_OFFSET_DEG + trim;
            requestedAngle = ControlMath.wrapDegrees(robotAngle - RobotConfig.TURRET_CENTER_HEADING_DEG);

            if (tracking && manualAngle == null && targetDistance >= .01) {
                setTargetPosition(robotAngle);
            }
        }

        holdFixedPosition();

        double maxStep = Math.max(0, RobotConfig.TURRET_SPEED_DEG_PER_SEC) * ControlMath.clamp(dt, 0, .1);
        estimatedAngle += ControlMath.clamp(targetAngle - estimatedAngle, -maxStep, maxStep);
    }

    private void holdFixedPosition() {
        servo.setPosition(FIXED_POSITION);
        secondServo.setPosition(FIXED_POSITION);
    }

    public void setTargetPosition(double robotAngleDegrees) {
        requestedAngle = ControlMath.wrapDegrees(robotAngleDegrees - RobotConfig.TURRET_CENTER_HEADING_DEG);
        TurretAim.LimitedAngle limited = TurretAim.limit(requestedAngle, estimatedAngle,
                RobotConfig.TURRET_MIN_DEG, RobotConfig.TURRET_MAX_DEG);
        targetAngle = heldAngle = limited.degrees;
        reachable = limited.reachable;
        validAim = true;
    }

    private static double clampToLimits(double degrees) {
        return ControlMath.clamp(degrees, RobotConfig.TURRET_MIN_DEG, RobotConfig.TURRET_MAX_DEG);
    }

    public static double toServoPosition(double turretDegrees) {
        double travel = Math.max(1e-6, RobotConfig.TURRET_SERVO_RANGE_DEG * RobotConfig.TURRET_GEAR_RATIO);
        return ControlMath.clamp(RobotConfig.TURRET_CENTER_POSITION + turretDegrees / travel, 0, 1);
    }

    public void aim() { tracking = true; }

    public void hold() { tracking = false; }
    public void nudge(double degrees) {
        trim += degrees;
        heldAngle = clampToLimits(heldAngle + degrees);
    }
    public void setManualAngle(Double degrees) { manualAngle = degrees; }
    public void reset() {
        tracking = false;
        trim = 0;
        heldAngle = 0;
    }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getFieldBearingDegrees() { return fieldBearing; }
    public double getRequestedAngleDegrees() { return requestedAngle; }

    public double getTargetAngleDegrees() { return targetAngle; }

    public double getEstimatedAngleDegrees() { return estimatedAngle; }

    public double getErrorDegrees() { return targetAngle - estimatedAngle; }

    public double getTrimDegrees() { return trim; }

    public TurretAim.HiveHalf getHiveHalf() { return half; }
    public double getTargetDistanceInches() { return targetDistance; }

    public boolean isTargetReachable() { return validAim && reachable; }

    public boolean isAimed() {
        return tracking && isTargetReachable()
                && Math.abs(getErrorDegrees()) <= RobotConfig.TURRET_TOLERANCE_DEG;
    }
}
