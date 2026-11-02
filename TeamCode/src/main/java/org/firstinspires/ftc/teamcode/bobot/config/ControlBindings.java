package org.firstinspires.ftc.teamcode.bobot.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public final class ControlBindings {
    private ControlBindings() { }

    public enum Button {
        A, B, X, Y, CROSS, CIRCLE, SQUARE, TRIANGLE, LEFT_BUMPER, RIGHT_BUMPER, LEFT_TRIGGER, RIGHT_TRIGGER,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, START, BACK, LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON
    }

    public static int DRIVE_DRIVER = 1;
    public static int SLOW_DRIVER = 1;
    public static Button SLOW_BUTTON = Button.RIGHT_TRIGGER;

    public static int SHOOT_DRIVER = 1;
    public static Button SHOOT_BUTTON = Button.TRIANGLE;
    public static int INTAKE_DRIVER = 1;
    public static Button INTAKE_BUTTON = Button.X;
    public static int EJECT_DRIVER = 1;
    public static Button EJECT_BUTTON = Button.B;
    public static int STOP_DRIVER = 1;
    public static Button STOP_BUTTON = Button.A;

    public static int TURRET_LEFT_DRIVER = 1;
    public static Button TURRET_LEFT_BUTTON = Button.LEFT_BUMPER;
    public static int TURRET_RIGHT_DRIVER = 1;
    public static Button TURRET_RIGHT_BUTTON = Button.RIGHT_BUMPER;
    public static int RESET_POSE_DRIVER = 1;
    public static Button RESET_POSE_BUTTON = Button.DPAD_RIGHT;
    public static int FLOWER_TAKER_DRIVER = 1;
    public static Button FLOWER_TAKER_BUTTON = Button.DPAD_UP;

    public static double TRIGGER_THRESHOLD = 0.2;

    public static Gamepad driver(int number, Gamepad one, Gamepad two) { return number == 2 ? two : one; }

    public static boolean held(int driver, Button button, Gamepad one, Gamepad two) {
        Gamepad g = driver(driver, one, two);
        switch (button) {
            case A: case CROSS: return g.a;
            case B: case CIRCLE: return g.b;
            case X: case SQUARE: return g.x;
            case Y: case TRIANGLE: return g.y;
            case LEFT_BUMPER: return g.left_bumper;
            case RIGHT_BUMPER: return g.right_bumper;
            case LEFT_TRIGGER: return g.left_trigger > TRIGGER_THRESHOLD;
            case RIGHT_TRIGGER: return g.right_trigger > TRIGGER_THRESHOLD;
            case DPAD_UP: return g.dpad_up;
            case DPAD_DOWN: return g.dpad_down;
            case DPAD_LEFT: return g.dpad_left;
            case DPAD_RIGHT: return g.dpad_right;
            case START: return g.start;
            case BACK: return g.back;
            case LEFT_STICK_BUTTON: return g.left_stick_button;
            case RIGHT_STICK_BUTTON: return g.right_stick_button;
            default: return false;
        }
    }
}
