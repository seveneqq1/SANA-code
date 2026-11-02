package org.firstinspires.ftc.teamcode.bobot.states;

import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;

public final class PoseStorage {
    private static Pose pose;
    private static Alliance alliance;
    private static long savedAt;

    private PoseStorage() { }

    public static void save(Pose current, Alliance currentAlliance) {
        if (!ControlMath.validPose(current)) return;
        pose = current;
        alliance = currentAlliance;
        savedAt = System.nanoTime();
    }

    public static Pose consume(Alliance selectedAlliance) {
        double ageSeconds = (System.nanoTime() - savedAt) / 1e9;
        boolean usable = pose != null && selectedAlliance == alliance
                && ageSeconds <= RobotConfig.POSE_HANDOFF_MAX_AGE_SECONDS;
        Pose result = usable ? pose : null;
        clear();
        return result;
    }

    public static void clear() {
        pose = null;
        alliance = null;
        savedAt = 0;
    }

    public static Pose cornerPose(Alliance selectedAlliance) {
        double heading = Math.toRadians(RobotConfig.RED_CORNER_HEADING_DEG);
        if (selectedAlliance == Alliance.RED) return new Pose(RobotConfig.RED_CORNER_X, RobotConfig.RED_CORNER_Y, heading);
        return new Pose(144 - RobotConfig.RED_CORNER_X, 144 - RobotConfig.RED_CORNER_Y, heading + Math.PI);
    }

    public static Pose startPose(Alliance selectedAlliance) {
        if (selectedAlliance == Alliance.BLUE) {
            return new Pose(RobotConfig.BLUE_START_X, RobotConfig.BLUE_START_Y,
                    Math.toRadians(RobotConfig.BLUE_START_HEADING_DEG));
        }
        return new Pose(RobotConfig.RED_START_X, RobotConfig.RED_START_Y,
                Math.toRadians(RobotConfig.RED_START_HEADING_DEG));
    }
}
