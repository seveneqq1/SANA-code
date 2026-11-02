package org.firstinspires.ftc.teamcode.bobot.utilities;

import org.firstinspires.ftc.teamcode.bobot.states.Alliance;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
public final class TurretAim {
    private TurretAim() { }

    public enum HiveHalf { LOWER, UPPER }

    public static final class LimitedAngle {
        public final double degrees;
        public final boolean reachable;

        LimitedAngle(double degrees, boolean reachable) {
            this.degrees = degrees;
            this.reachable = reachable;
        }
    }

    public static Pose hive(Alliance alliance, HiveHalf half) {
        boolean lower = half == HiveHalf.LOWER;
        if (alliance == Alliance.BLUE) {
            return lower
                    ? new Pose(RobotConfig.BLUE_LOWER_TARGET_X, RobotConfig.BLUE_LOWER_TARGET_Y)
                    : new Pose(RobotConfig.BLUE_UPPER_TARGET_X, RobotConfig.BLUE_UPPER_TARGET_Y);
        }
        return lower
                ? new Pose(RobotConfig.RED_LOWER_TARGET_X, RobotConfig.RED_LOWER_TARGET_Y)
                : new Pose(RobotConfig.RED_UPPER_TARGET_X, RobotConfig.RED_UPPER_TARGET_Y);
    }
    public static Pose pivot(Pose stateController) {
        double c = Math.cos(stateController.heading()), s = Math.sin(stateController.heading());
        double forward = RobotConfig.TURRET_PIVOT_FORWARD_IN, left = RobotConfig.TURRET_PIVOT_LEFT_IN;
        return new Pose(stateController.x() + c * forward - s * left, stateController.y() + s * forward + c * left, stateController.heading());
    }
    public static HiveHalf select(Pose pivot, HiveHalf previous) {
        double split = RobotConfig.TARGET_SPLIT_Y, hysteresis = Math.max(0, RobotConfig.TARGET_SWITCH_HYSTERESIS_IN);
        if (previous == HiveHalf.LOWER && pivot.y() <= split + hysteresis) return HiveHalf.LOWER;
        if (previous == HiveHalf.UPPER && pivot.y() > split - hysteresis) return HiveHalf.UPPER;
        return pivot.y() <= split ? HiveHalf.LOWER : HiveHalf.UPPER;
    }
    public static double relativeDegrees(Pose pivot, Pose target, double offsetDeg) {
        double fieldBearing = Math.atan2(target.y() - pivot.y(), target.x() - pivot.x());
        return ControlMath.wrapDegrees(Math.toDegrees(fieldBearing - pivot.heading()) + offsetDeg);
    }
    public static Pose movingShotTarget(Pose target, double velocityX, double velocityY, double flightSeconds) {
        if (!Double.isFinite(velocityX) || !Double.isFinite(velocityY) || !Double.isFinite(flightSeconds)) return target;
        double time = Math.max(0, flightSeconds);
        return new Pose(target.x() - velocityX * time, target.y() - velocityY * time, target.heading());
    }
    public static LimitedAngle limit(double desired, double current, double min, double max) {
        double best = ControlMath.clamp(desired, min, max), bestDistance = Double.POSITIVE_INFINITY;
        boolean found = false;
        for (int turn = -2; turn <= 2; turn++) {
            double candidate = desired + turn * 360;
            boolean reachable = candidate >= min && candidate <= max;
            double limited = ControlMath.clamp(candidate, min, max);
            double distance = Math.abs(limited - current);
            if ((reachable && !found) || (reachable == found && distance < bestDistance)) {
                best = limited;
                bestDistance = distance;
                found = reachable;
            }
        }
        return new LimitedAngle(best, found);
    }
}
