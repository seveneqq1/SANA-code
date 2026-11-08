package org.firstinspires.ftc.teamcode.pedro;

/**
 * Pedro Pathing 3 settings for this season's robot.
 *
 * create() is not finished: it still returns null so TeleOp that only needs
 * Pinpoint heading (RegionalATeleOp) can run. Fill it in after you finish
 * the Pinpoint tuner (signed offsets from the center of rotation) and the
 * mecanum drive names in a Follower builder.
 */

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
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

    public static Follower create(HardwareMap h) {
        // TODO: build a Pedro 3 Follower here after Pinpoint offsets and drive
        // constants are measured. Callers must null-check until this is done.
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
}
