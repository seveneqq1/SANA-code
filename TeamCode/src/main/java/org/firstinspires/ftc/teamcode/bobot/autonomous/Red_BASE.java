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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Red_BASE", group = "!competition")
public class Red_BASE extends OpMode {
    private Timer opmodeTimer;
    private Follower follower;
    private PathChain scorePreload, intake_first_line, score_first_line, intake_second, score_second, intake_gate, score_gate, intake_gate_2, score_gate_2, intake_gate_3, score_gate_3, intake_gate_4, score_gate_4, park;

    public enum Path_States {
        SCORE_PRELOAD,
        INTAKE_SECOND, SCORE_SECOND,
        INTAKE_GATE, WAIT_FOR_THE_GATE, SCORE_GATE,
        INTAKE_FIRST, SCORE_FIRST,
        INTAKE_GATE_2, WAIT_FOR_THE_GATE_2, SCORE_GATE_2,
        INTAKE_GATE_3, WAIT_FOR_THE_GATE_3, SCORE_GATE_3,
        INTAKE_GATE_4, WAIT_FOR_THE_GATE_4, SCORE_GATE_4,
        PARK, STOP
    }

    private Path_States pathStates;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(121.000, 111.000),
                               new Pose(110,116),
                               new Pose(88,88)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        intake_first_line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 88.000),
                                new Pose(120.000, 85.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score_first_line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(115.000, 85.000),
                                new Pose(88.000, 88.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake_second = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88,88),
                                new Pose(83, 55),
                                new Pose(111, 59)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();

        score_second = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(111, 59),
                                new Pose(88.000, 80.000)
                        )
                )
                .setLinearHeadingInterpolation(0,Math.toRadians(331))
                .build();
        intake_gate=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88,80),
                                new Pose(88.232, 52.240),
                                new Pose(106.363, 63.849),
                                new Pose(Pose_states.open_gate_red.getX(),Pose_states.open_gate_red.getY())                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(331),Pose_states.open_gate_red.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.open_gate_red,
                                new Pose(90,58),
                                new Pose(85,80)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        intake_gate_2=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85,80),
                                new Pose(88.232, 52.240),
                                new Pose(106.363, 63.849),
                                new Pose(Pose_states.open_gate_red.getX()+1,Pose_states.open_gate_red.getY())
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(331),Pose_states.open_gate_red.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate_2=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.open_gate_red,
                                new Pose(90,58),
                                new Pose(88,80)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        intake_gate_3=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88,80),
                                new Pose(88.232, 52.240),
                                new Pose(106.363, 63.849),
                                new Pose(Pose_states.open_gate_red.getX()+2,Pose_states.open_gate_red.getY())
                        )
                )
                .setConstantHeadingInterpolation(Pose_states.open_gate_red.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate_3=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.open_gate_red,
                                new Pose(90,58),
                                new Pose(88,80)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        intake_gate_4=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88,80),
                                new Pose(88.232, 52.240),
                                new Pose(106.363, 63.849),
                                new Pose(Pose_states.open_gate_red.getX()+3,Pose_states.open_gate_red.getY())
                        )
                )
                .setConstantHeadingInterpolation(Pose_states.open_gate_red.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate_4=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.open_gate_red,
                                new Pose(90,58),
                                new Pose(88,80)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        park=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Pose_states.score_pose_for_blue,
                                Pose_states.startPose_Red_TeleOp                       )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private ElapsedTime timer;

    public void autonomousPathUpdate() {
        switch (pathStates) {
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                follower.setMaxPower(0.7);
                setPathState(Path_States.INTAKE_SECOND);
                break;
            case INTAKE_SECOND:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(intake_second);
                    follower.setMaxPower(1);
                    setPathState(Path_States.SCORE_SECOND);
                }
                break;
            case SCORE_SECOND:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(score_second);
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE);
                }
                break;
            case INTAKE_GATE:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(intake_gate);
                    follower.setMaxPower(1);
                    setPathState(Path_States.WAIT_FOR_THE_GATE);
                }
                break;
            case WAIT_FOR_THE_GATE:
                if (follower.atPose(Pose_states.open_gate_red, 5, 5)) {
                    setPathState(Path_States.SCORE_GATE);
                    timer.reset();
                }
                break;
            case SCORE_GATE:
                if (timer.milliseconds() > 500) {
                    follower.followPath(score_gate);
                    setPathState(Path_States.INTAKE_FIRST);
                }
                break;
            case INTAKE_FIRST:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(intake_first_line);
                    follower.setMaxPower(1);
                    setPathState(Path_States.SCORE_FIRST);
                }
                break;
            case SCORE_FIRST:
                if (follower.getPose().getX() > 115 || !follower.isBusy()) {
                    follower.followPath(score_first_line);
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE_2);
                }
                break;
            case INTAKE_GATE_2:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(intake_gate_2);
                    follower.setMaxPower(1);
                    setPathState(Path_States.WAIT_FOR_THE_GATE_2);
                }
                break;
            case WAIT_FOR_THE_GATE_2:
                if (follower.atPose(Pose_states.open_gate_red, 5, 5)) {
                    setPathState(Path_States.SCORE_GATE_2);
                    timer.reset();
                }
                break;
            case SCORE_GATE_2:
                if (timer.milliseconds() > 500) {
                    follower.followPath(score_gate_2);
                    setPathState(Path_States.INTAKE_GATE_3);
                }
                break;
            case INTAKE_GATE_3:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(intake_gate_3);
                    follower.setMaxPower(1);
                    setPathState(Path_States.WAIT_FOR_THE_GATE_3);
                }
                break;
            case WAIT_FOR_THE_GATE_3:
                if (follower.atPose(Pose_states.open_gate_red, 6, 6)) {
                    setPathState(Path_States.SCORE_GATE_3);
                    timer.reset();
                }
                break;
            case SCORE_GATE_3:
                if (timer.milliseconds() > 500) {
                    follower.followPath(score_gate_3);
                    setPathState(Path_States.INTAKE_GATE_4);
                }
                break;
            case INTAKE_GATE_4:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(intake_gate_4);
                    follower.setMaxPower(1);
                    setPathState(Path_States.WAIT_FOR_THE_GATE_4);
                }
                break;
            case WAIT_FOR_THE_GATE_4:
                if (follower.atPose(Pose_states.open_gate_red, 8, 8)) {
                    setPathState(Path_States.SCORE_GATE_4);
                    timer.reset();
                }
                break;
            case SCORE_GATE_4:
                if (timer.milliseconds() > 500) {
                    follower.followPath(score_gate_4);
                    setPathState(Path_States.PARK);
                }
                break;
            case PARK:
                if (follower.getCurrentTValue() > 0.98 || !follower.isBusy()) {
                    follower.followPath(park);
                    follower.setMaxPower(1);
                    setPathState(Path_States.STOP);
                }
                break;
            case STOP:
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(Path_States pathState) {
        this.pathStates = pathState;
    }

    private List<LynxModule> allHubs;

    @Override
    public void init() {
        timer = new ElapsedTime();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(121, 111, 0));
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(Path_States.SCORE_PRELOAD);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathStates);
        telemetry.update();
    }
}
