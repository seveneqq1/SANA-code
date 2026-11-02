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

import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(group = "!competition")
public class Just_Constants extends OpMode {
    private Timer opmodeTimer;
    private Follower follower;
    private final Pose score_pose_for_preload=new Pose(108,108,Math.toRadians(270));
    private PathChain scorePreload, intake_first,score_first, intake_second, score_second, intake_gate,score_gate, intake_gate_2,score_gate_2;
    public enum Path_States{
        ZERO,
        SCORE_PRELOAD,
        INTAKE_FIRST,SCORE_FIRST,
        INTAKE_SECOND, SCORE_SECOND,
        INTAKE_GATE,SCORE_GATE,
        INTAKE_GATE_2,SCORE_GATE_2,
        SHOOT;
    }
    private Path_States pathStates= Path_States.ZERO;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.000, 116.000),
                                score_pose_for_preload
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        intake_first = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(108.000, 108.000),
                                new Pose(116.464, 104.557),
                                new Pose(112.000, 89.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        score_first = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(112.000, 89.000),
                                new Pose(108.000, 108.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake_second = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(108.000, 108.000),
                                new Pose(116.961, 94.468),
                                new Pose(112.000, 68.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        score_second = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(112.000, 68.000),
                                new Pose(88.000, 88.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();
        intake_gate=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88,88),
                                new Pose(104,39),
                                Pose_states.open_gate_red
                        )
                )
                .setConstantHeadingInterpolation(Pose_states.open_gate_red.getHeading())
                .build();
        score_gate=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.open_gate_red,
                                new Pose(90,58),
                                new Pose(88,88)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),0)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathStates) {
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                setPathState(Path_States.INTAKE_FIRST);
                break;
            case INTAKE_FIRST:
                if(!follower.isBusy()){
                follower.followPath(intake_first);
                follower.setMaxPower(0.3);
                setPathState(Path_States.SCORE_FIRST);}
                break;
            case SCORE_FIRST:
                if(follower.getCurrentTValue()>0.99){
                    follower.followPath(score_first);
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_SECOND);
                }
                break;
            case INTAKE_SECOND:
                if(!follower.isBusy()){
                follower.followPath(intake_second);
                if(follower.getCurrentTValue()>0.4){
                    follower.setMaxPower(0.3);
                }
                else {
                follower.setMaxPower(0.5);
                }
                setPathState(Path_States.SCORE_SECOND);}
                break;
            case SCORE_SECOND:
                if(follower.getCurrentTValue()>0.99){
                    follower.followPath(score_second);
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE);
                }
                break;
            case INTAKE_GATE:
                if(!follower.isBusy()){
                    follower.followPath(intake_gate);
                    setPathState(Path_States.SCORE_GATE);
                }
                break;
            case SCORE_GATE:
                if(!follower.isBusy()){
                    follower.followPath(score_gate);
                    setPathState(Path_States.INTAKE_GATE);
                }
        }
    }
     
    public void setPathState(Path_States pathState) {
        this.pathStates=pathState;
    }
    private List<LynxModule> allHubs;
    @Override
    public void init() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(122.8177,112,Math.toRadians(270)));
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

    }
    @Override
    public void init_loop() {}
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(Path_States.SCORE_PRELOAD);
    }
    private double lastTimeStamp=0;
     
    @Override
    public void loop() {
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0)
            lastTimeStamp = currentTimeStamp;
        double dt = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;
        double frequency = 1 / dt;
        telemetry.addData("frequency", frequency);
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void stop() {}
}