package org.firstinspires.ftc.teamcode.bobot.utilities;

import com.pedropathing.math.Pose;

public final class ControlMath {
    private ControlMath() { }
    public static double clamp(double value, double min, double max) {
        if (!Double.isFinite(value) || !Double.isFinite(min) || !Double.isFinite(max)) return 0;
        return Math.max(min, Math.min(max, value));
    }
    public static double wrapDegrees(double degrees) {
        double radians = Math.toRadians(degrees);
        return Math.toDegrees(Math.atan2(Math.sin(radians), Math.cos(radians)));
    }
    public static boolean validPose(Pose pose) {
        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    public static double interpolate(double[] xs, double[] ys, double x) {
        int n = Math.min(xs.length, ys.length);
        if (n == 0 || !Double.isFinite(x)) return Double.NaN;
        if (x <= xs[0]) return ys[0];
        for (int i = 1; i < n; i++) {
            if (x <= xs[i]) {
                double span = xs[i] - xs[i - 1];
                double t = span > 0 ? (x - xs[i - 1]) / span : 1;
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        return ys[n - 1];
    }
    public static double deadband(double value, double threshold) {
        threshold = clamp(threshold, 0, 0.99);
        if (Math.abs(value) <= threshold) return 0;
        return Math.copySign((Math.abs(value) - threshold) / (1 - threshold), value);
    }
}
