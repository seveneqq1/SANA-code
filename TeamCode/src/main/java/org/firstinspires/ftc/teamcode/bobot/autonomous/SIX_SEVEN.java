package org.firstinspires.ftc.teamcode.bobot.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterControllerjana;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateControllerAuto;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
public class SIX_SEVEN {
    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.000, 110.000),
                                    new Pose(39.000, 106.000),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierLine(
                                    new Pose(58.000, 83.000),
                                    new Pose(20.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(20.000, 83.000),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 83.000),
                                    new Pose(70.994, 58.012),
                                    new Pose(21.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.000, 59.000),
                                    new Pose(47.000, 47.006),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 83.000),
                                    new Pose(47.000, 58.000),
                                    new Pose(11.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.000, 58.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(57.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.000, 83.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(11.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.000, 58.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(57.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.000, 83.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(11.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.000, 58.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 83.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(11.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.000, 58.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 83.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(11.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(150))
                    .addPath(
                            new BezierCurve(
                                    new Pose(11.000, 58.000),
                                    new Pose(24.000, 47.000),
                                    new Pose(58.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(135))
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 83.000),
                                    new Pose(63.433, 83.937),
                                    new Pose(103.000, 47.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }
}