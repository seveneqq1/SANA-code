package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;

public class PillarAvoidance {
    public static final class Output {
        public final double x, y, turn;
        public final boolean avoiding;

        Output(double x, double y, double turn, boolean avoiding) {
            this.x = x;
            this.y = y;
            this.turn = turn;
            this.avoiding = avoiding;
        }
    }

    public double getClearanceRadius() {
        return Math.max(0, RobotConfig.ROBOT_RADIUS_IN) + Math.max(0, RobotConfig.PILLAR_RADIUS_IN)
                + Math.max(0, RobotConfig.AVOIDANCE_MARGIN_IN);
    }
    public Pose[] getObstaclePoints(Pose stateController) {
        Pose[] feet = {
                new Pose(RobotConfig.PILLAR_LEFT_X, RobotConfig.PILLAR_BOTTOM_Y),
                new Pose(RobotConfig.PILLAR_RIGHT_X, RobotConfig.PILLAR_BOTTOM_Y),
                new Pose(RobotConfig.PILLAR_LEFT_X, RobotConfig.PILLAR_TOP_Y),
                new Pose(RobotConfig.PILLAR_RIGHT_X, RobotConfig.PILLAR_TOP_Y)
        };
        double centerX = (RobotConfig.PILLAR_LEFT_X + RobotConfig.PILLAR_RIGHT_X) / 2;
        double legFraction = ControlMath.clamp(RobotConfig.ROBOT_HEIGHT_IN
                / Math.max(1, RobotConfig.FRAME_PIVOT_HEIGHT_IN), 0, 1);

        Pose[] points = new Pose[6];
        points[0] = nearestOnSegment(stateController, feet[0], feet[1]);
        points[1] = nearestOnSegment(stateController, feet[2], feet[3]);
        for (int i = 0; i < feet.length; i++) {
            double pivotY = i < 2 ? RobotConfig.FRAME_PIVOT_BOTTOM_Y : RobotConfig.FRAME_PIVOT_TOP_Y;
            Pose legTop = new Pose(feet[i].x() + legFraction * (centerX - feet[i].x()),
                    feet[i].y() + legFraction * (pivotY - feet[i].y()));
            points[i + 2] = nearestOnSegment(stateController, feet[i], legTop);
        }
        return points;
    }

    private static Pose nearestOnSegment(Pose point, Pose start, Pose end) {
        double dx = end.x() - start.x(), dy = end.y() - start.y();
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared < 1e-9 ? 0 : ControlMath.clamp(((point.x() - start.x()) * dx
                + (point.y() - start.y()) * dy) / lengthSquared, 0, 1);
        return new Pose(start.x() + t * dx, start.y() + t * dy);
    }
    public Output apply(Pose pose, double x, double y, double turn, boolean bypass) {
        if (!ControlMath.validPose(pose) || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(turn)) {
            return new Output(0, 0, 0, true);
        }
        if (bypass || !RobotConfig.AVOIDANCE_ENABLED) return new Output(x, y, turn, false);
        double speed = Math.hypot(x, y);
        if (speed < 1e-6) return new Output(0, 0, turn, false);

        double originalX = x, originalY = y;
        double radius = getClearanceRadius();
        double maxSpeed = Math.max(1, RobotConfig.AVOIDANCE_MAX_SPEED_IPS);
        double horizon = Math.max(0.05, RobotConfig.AVOIDANCE_LOOKAHEAD_SECONDS);
        double deceleration = Math.max(1, RobotConfig.AVOIDANCE_DECELERATION_IPS2);
        double lookahead = maxSpeed * speed * horizon + maxSpeed * maxSpeed * speed * speed / (2 * deceleration);
        double cap = RobotConfig.AVOIDANCE_MAX_LOOKAHEAD_IN;
        if (Double.isFinite(cap) && cap > 0) lookahead = Math.min(lookahead, cap);
        Pose[] obstacles = getObstaclePoints(pose);
        Pose threat = null;
        double threatDistance = Double.POSITIVE_INFINITY;
        for (Pose obstacle : obstacles) {
            double dx = obstacle.x() - pose.x(), dy = obstacle.y() - pose.y();
            double along = (dx * x + dy * y) / speed;
            double cross = Math.abs(dx * y - dy * x) / speed;
            if (along > 0 && along < lookahead + radius && cross < radius && along < threatDistance) {
                threat = obstacle;
                threatDistance = along;
            }
        }
        if (threat != null) {
            double dx = pose.x() - threat.x(), dy = pose.y() - threat.y();
            double distance = Math.max(1e-6, Math.hypot(dx, dy));
            double tx = -dy / distance, ty = dx / distance;
            double sign = x * tx + y * ty < -1e-6 ? -1 : 1;
            double influence = ControlMath.clamp(1 - Math.max(0, distance - radius) / lookahead, 0, 1);
            double steer = sign * speed * influence * Math.max(0, RobotConfig.AVOIDANCE_STEER_GAIN);
            x += tx * steer;
            y += ty * steer;
        }

        for (int pass = 0; pass < 8; pass++) {
            for (Pose obstacle : obstacles) {
                double dx = pose.x() - obstacle.x(), dy = pose.y() - obstacle.y();
                double distance = Math.hypot(dx, dy);
                if (distance < 1e-6) return new Output(0, 0, 0, true);
                double nx = dx / distance, ny = dy / distance;
                double limit = approachLimit(distance - radius, maxSpeed, horizon, deceleration);
                double dot = x * nx + y * ny;
                if (dot < -limit) {
                    x += (-limit - dot) * nx;
                    y += (-limit - dot) * ny;
                }
            }
        }

        for (Pose obstacle : obstacles) {
            double dx = pose.x() - obstacle.x(), dy = pose.y() - obstacle.y();
            double distance = Math.hypot(dx, dy);
            double limit = approachLimit(distance - radius, maxSpeed, horizon, deceleration);
            if ((x * dx + y * dy) / distance < -limit - 1e-6) {
                x = 0;
                y = 0;
                break;
            }
        }

        double adjustedSpeed = Math.hypot(x, y);
        if (adjustedSpeed > speed) {
            x *= speed / adjustedSpeed;
            y *= speed / adjustedSpeed;
        }
        if (!Double.isFinite(x) || !Double.isFinite(y)) return new Output(0, 0, 0, true);
        return new Output(x, y, turn, Math.abs(x - originalX) + Math.abs(y - originalY) > 1e-5);
    }

    private static double approachLimit(double gap, double maxSpeed, double horizon, double deceleration) {
        gap = Math.max(0, gap);
        return Math.min(gap / (maxSpeed * horizon), Math.sqrt(2 * deceleration * gap) / maxSpeed);
    }
}
