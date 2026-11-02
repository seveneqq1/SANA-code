package org.firstinspires.ftc.teamcode.bobot.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public final class RobotConfig {
    private RobotConfig() { }

    public static String INTAKE_NAME = "intake";
    public static String TRANSFER_NAME = "transfer";
    public static String SHOOTER_LEFT_NAME = "shooter1";
    public static String SHOOTER_RIGHT_NAME = "shooter2";
    public static String TURRET_NAME = "turret";
    public static String TURRET_SECOND_NAME = "turret1";
    public static DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction SHOOTER_LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction SHOOTER_RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static boolean TURRET_REVERSED = true;
    public static boolean TURRET_SECOND_REVERSED = true;

    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 14.0;
    public static double VOLTAGE_MAX_SCALE = 1.64;
    public static double VOLTAGE_READ_INTERVAL_MS = 100;
    public static double VOLTAGE_FILTER_ALPHA = 0.3;
    public static double VOLTAGE_MIN_VALID = 6.0;

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_EJECT_POWER = -1.0;
    public static double TRANSFER_COLLECT_POWER = 0.35;
    public static double TRANSFER_EJECT_POWER = -0.8;

    public static double TURRET_SERVO_RANGE_DEG = 330.0;
    public static double TURRET_GEAR_RATIO = 1.5;
    public static double TURRET_CENTER_POSITION = 0.5;
    public static double TURRET_CENTER_HEADING_DEG = 180.0;
    public static double TURRET_MIN_DEG = -247.5;
    public static double TURRET_MAX_DEG = 247.5;
    public static double TURRET_SPEED_DEG_PER_SEC = 600.0;
    public static double TURRET_TOLERANCE_DEG = 2.0;
    public static double TURRET_NUDGE_DEG = 2.0;
    public static double TURRET_OFFSET_DEG = 8;
    public static double TURRET_PIVOT_FORWARD_IN = 0;
    public static double TURRET_PIVOT_LEFT_IN = 0.0;

    public static String STOPPER_NAME = "stopper";

    public static String FLOWER_TAKER_1_NAME = "fl1";
    public static String FLOWER_TAKER_2_NAME = "fl2";
    public static double FLOWER_TAKER_1_OPEN_POSITION = 0.5;
    public static double FLOWER_TAKER_1_CLOSED_POSITION = 1;
    public static double FLOWER_TAKER_2_OPEN_POSITION = 0.5;
    public static double FLOWER_TAKER_2_CLOSED_POSITION = 0.0;

    public static double TARGET_SPLIT_Y = 62.0;
    public static double TARGET_SWITCH_HYSTERESIS_IN = 2.0;
    public static double RED_LOWER_TARGET_X = 60.0;
    public static double RED_LOWER_TARGET_Y = 52;
    public static double RED_UPPER_TARGET_X = 60.0;
    public static double RED_UPPER_TARGET_Y = 72;
    public static double BLUE_LOWER_TARGET_X = 84.0;
    public static double BLUE_LOWER_TARGET_Y = 52;
    public static double BLUE_UPPER_TARGET_X = 84.0;
    public static double BLUE_UPPER_TARGET_Y = 72;

    public static double PILLAR_LEFT_X = 72.0 - 38.95 / 2;
    public static double PILLAR_RIGHT_X = 72.0 + 38.95 / 2;
    public static double PILLAR_BOTTOM_Y = 72.0 - 49.46 / 2;
    public static double PILLAR_TOP_Y = 72.0 + 49.46 / 2;
    public static double FRAME_PIVOT_HEIGHT_IN = 43.95;
    public static double FRAME_PIVOT_BOTTOM_Y = 59.25;
    public static double FRAME_PIVOT_TOP_Y = 84.75;

    public static boolean AVOIDANCE_ENABLED = false;
    public static double AVOIDANCE_MAX_LOOKAHEAD_IN = 28.0;
    public static double ROBOT_HEIGHT_IN = 18.0;
    public static double PILLAR_RADIUS_IN = 2.0;
    public static double ROBOT_RADIUS_IN = 13.0;
    public static double AVOIDANCE_MARGIN_IN = 2.0;
    public static double AVOIDANCE_LOOKAHEAD_SECONDS = 0.45;
    public static double AVOIDANCE_MAX_SPEED_IPS = 82.0;
    public static double AVOIDANCE_DECELERATION_IPS2 = 34.0;
    public static double AVOIDANCE_STEER_GAIN = 0.7;

    public static double DRIVE_DEADBAND = 0.05;
    public static double DRIVE_SLOW_SCALE = 0.3;
    public static double BLUE_START_X = 108.0;
    public static double BLUE_START_Y = 132.0;
    public static double BLUE_START_HEADING_DEG = 270.0;
    public static double RED_START_X = 36.0;
    public static double RED_START_Y = 12.0;
    public static double RED_START_HEADING_DEG = 90.0;

    public static double RED_CORNER_X = 9.0;
    public static double RED_CORNER_Y = 9.0;
    public static double RED_CORNER_HEADING_DEG = 90.0;
    public static boolean TELEOP_USE_AUTO_POSE = true;
    public static double POSE_HANDOFF_MAX_AGE_SECONDS = 120.0;

    public static double SHOOT_FLIGHT_TIME_SECONDS = 0.0;

    public static double SHOOT_MAX_TRACKING_SPEED_IPS = 120.0;
    public static double AUTO_READY_TIMEOUT_MS = 4000;
    public static double AUTO_FEED_MS = 2000;
}
