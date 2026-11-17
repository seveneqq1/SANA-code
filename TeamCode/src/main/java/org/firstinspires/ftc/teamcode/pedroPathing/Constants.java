package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;

/** Robot calibration and live tuning values. Positions use inches unless marked otherwise. */
public final class Constants {
    private Constants() {}

    public static final String FRONT_LEFT = "frontleft";
    public static final String FRONT_RIGHT = "frontright";
    public static final String BACK_LEFT = "backleft";
    public static final String BACK_RIGHT = "backright";
    public static final String PINPOINT_NAME = "pinpoint";

    public static final double TRIGGER_DEADBAND = 0.2;

    /**
     * CAD measurement from Fusion assembly v18, recorded 2026-11-08.
     *
     * These are unsigned, center-to-center distances between the two odometry modules,
     * not Pinpoint xPodOffset/yPodOffset values. Pedro uses +X forward and +Y left,
     * and requires each pod's signed offset from the robot's center of rotation.
     * Do not pass these values to Pinpoint#setOffsets until that center-relative
     * measurement and the pod directions have been verified on the robot.
     */
    public static final double ODO_MODULE_LONGITUDINAL_SEPARATION_MM = 14.2;
    public static final double ODO_MODULE_TRANSVERSE_SEPARATION_MM = 335.8;
    public static final double ODO_MODULE_CENTER_DISTANCE_MM = 336.1;

    // Pedro's default distance unit is inches; retain converted values beside the CAD source values.
    public static final double ODO_MODULE_LONGITUDINAL_SEPARATION_INCHES =
            ODO_MODULE_LONGITUDINAL_SEPARATION_MM / 25.4;
    public static final double ODO_MODULE_TRANSVERSE_SEPARATION_INCHES =
            ODO_MODULE_TRANSVERSE_SEPARATION_MM / 25.4;
    public static final double ODO_MODULE_CENTER_DISTANCE_INCHES =
            ODO_MODULE_CENTER_DISTANCE_MM / 25.4;

    @Configurable
    public static final class Snap {
        public static double START_X = 0, START_Y = 0, START_HEADING = 0;
        public static double SLOW_POWER = 0.35;
        public static double TURRET_DIRECTION = 1;
        public static double HEADING_KP = 1;
        public static double HEADING_KD = 0;
        public static double HEADING_MAX_POWER = 0.35;
        public static double HEADING_TOLERANCE_RADIANS = Math.toRadians(2);
        public static double RIGHT_STICK_DEADBAND = 0.05;

        private Snap() {}
    }

    @Configurable
    public static final class Tracking {
        public static double START_X = 0, START_Y = 0, START_HEADING = 0;
        public static double SLOW_POWER = 0.35;
        public static double TURRET_DIRECTION = 1;
        public static double HEADING_KP = 1;
        public static double HEADING_KD = 0;
        public static double HEADING_MAX_POWER = 0.35;
        public static double HEADING_TOLERANCE_RADIANS = Math.toRadians(2);
        public static double RIGHT_STICK_DEADBAND = 0.05;
        public static final double AIM_TRIM_DEGREES = 5;

        private Tracking() {}
    }

    @Configurable
    public static final class PointTrack {
        public static double TARGET_X = 110, TARGET_Y = 63;
        public static double ROBOT_LENGTH = 18, ROBOT_WIDTH = 18; // inches
        public static double TURRET_DIRECTION = 1;          // servo position up = turret turns left (measured Nov 14)
        public static double MANUAL_SPEED_DEG_PER_S = 120;
        public static double TICKS_PER_MM = 19.894;
        public static boolean REVERSE_X_POD = true;
        public static boolean REVERSE_Y_POD = false;
        public static double X_POD_OFFSET_MM = -154.5; // measured with Odo Offset Calibrate (Nov 13)
        public static double Y_POD_OFFSET_MM = -165.5; // measured with Odo Offset Calibrate (Nov 13)
        public static double SLOW_MULTIPLIER = 0.6;  // R2, same as Worlds
        public static double SHOOT_TIME = 1.5; // seconds stopper stays open; bigger if not all balls fly out
        public static boolean ALWAYS_SPIN = true;
        public static final double FIELD_SIZE_INCHES = 144;
        public static final double RESET_HEADING_DEGREES = 90;
        public static final double MAX_LOOP_SECONDS = 0.1;

        private PointTrack() {}
    }

    @Configurable
    public static final class LegacyTest {
        public static double START_X = 0;
        public static double START_Y = 0;
        public static double START_HEADING = 0;
        public static double SLOW_POWER = 0.35;
        public static final double SLOW_TRIGGER_THRESHOLD = 0.2;
        public static double TURRET_FORWARD_RADIANS = 0;
        public static double TURRET_DIRECTION = 1;

        private LegacyTest() {}
    }

    public static final class Diagnostics {
        public static final double STOPPER_BLOCK_POSITION = 0.35;
        public static final double STOPPER_FEED_POSITION = 0.5;
        public static final double TURRET_TEST_CENTER = 0.5;
        public static final double TURRET_TEST_STEP = 0.003;
        public static final long TURRET_TEST_INTERVAL_MS = 10;
        public static final double ODOMETRY_TURN_POWER = 0.4;
        public static final double TRANSFER_TEST_FEED_POWER = -0.8;

        private Diagnostics() {}
    }
}
