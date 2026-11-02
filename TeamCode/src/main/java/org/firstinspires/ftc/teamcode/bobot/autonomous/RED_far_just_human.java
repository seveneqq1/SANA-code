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

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterControllerjana;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateControllerAuto;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(group = "!competition")
public class RED_far_just_human extends OpMode {
    private Timer opmodeTimer;
    private Follower follower;
    IntakeController intakeController;
    ShooterControllerjana shooterControllerjana;
    TurretController turretController;
    StateControllerAuto stateController;
    private final Pose startPose=new Pose(86,9,0);
    private PathChain scorePreload, intake_third_line, score_third_line, intake_human,score_human, intake_human_again,score_human_again, intake_incoming_balls, score_incoming_balls;
    public enum Path_States{
        ZERO,
        SCORE_PRELOAD,
        INTAKE_HUMAN,SCORE_HUMAN,
        INTAKE_HUMAN_AGAIN,SCORE_HUMAN_AGAIN,
        INTAKE_THIRD,SCORE_THIRD,
        INTAKE_UPCOMING_BALLS,SCORE_UPCOMING_BALLS,
        SHOOT;
    }
    private Path_States pathStates= Path_States.ZERO;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                new Pose(81,13)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        intake_human=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(81,13),
                                new Pose(132,7)
                        )
                )
                .setTimeoutConstraint(1000)
                .build();
        score_human=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132,9),
                                new Pose(83,23)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        intake_human_again=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83,23),
                                new Pose(132,7)
                        )
                )
                .setTimeoutConstraint(1000)
                .build();
        score_human_again=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132,9),
                                new Pose(83,23)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        intake_incoming_balls=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83,23),
                                new Pose(132,25)
                        )
                )
                .setTimeoutConstraint(500)
                .build();
        score_incoming_balls=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(132,25),
                                new Pose(83,23)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        intake_third_line=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(81,23),
                                new Pose(82,44),
                                new Pose(119,35)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
        score_third_line=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(119,35),
                                new Pose(83,23)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
    }
    private double prevState=0;
    public void shoot(){
        stateController.shoot_whereever_you_want_auto();
        follower.setMaxPower(0.4);
    }
    public boolean is_shoot_ended(){
        return stateController.getState()== StateControllerAuto.State.SHOOT_ENDED_AUTO;
    }
    public void autonomousPathUpdate() {
        switch (pathStates) {
            case SHOOT:
                if(!follower.isBusy()){
                    shoot();
                    if(prevState==1 || prevState==2 || prevState==3 || prevState==4 || prevState==5 || prevState ==6 || prevState==7){
                        if(prevState==1 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_HUMAN);
                        }
                        if(prevState==2 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_THIRD);
                        }
                        if(prevState==3 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_UPCOMING_BALLS);
                        }
                        if(prevState==4 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_HUMAN_AGAIN);
                        }
                        if(prevState==5 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_UPCOMING_BALLS);
                        }
                    }
                }
                break;
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                follower.setMaxPower(1);
                prevState=1;
                setPathState(Path_States.SHOOT);
                break;
            case INTAKE_HUMAN:
                follower.followPath(intake_human);
                follower.setMaxPower(1);
                setPathState(Path_States.SCORE_HUMAN);
                break;
            case SCORE_HUMAN:
                if(follower.getCurrentTValue()>0.95){
                    follower.followPath( score_human);
                    follower.setMaxPower(1);
                    prevState=3;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_UPCOMING_BALLS:
                follower.followPath(intake_incoming_balls);
                follower.setMaxPower(1);
                setPathState(Path_States.SCORE_UPCOMING_BALLS);
                break;
            case SCORE_UPCOMING_BALLS:
                if(follower.getCurrentTValue()>0.97){
                    follower.followPath(score_incoming_balls);
                    follower.setMaxPower(1);
                    prevState=4;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_HUMAN_AGAIN:
                follower.followPath(intake_human_again);
                follower.setMaxPower(1);
                setPathState(Path_States.SCORE_HUMAN_AGAIN);
                break;
            case SCORE_HUMAN_AGAIN:
                if(follower.getCurrentTValue()>0.97){
                    follower.followPath(score_human_again);
                    follower.setMaxPower(1);
                    prevState=5;
                    setPathState(Path_States.SHOOT);
                }
        }
    }
     
    public void setPathState(Path_States pathState) {
        this.pathStates=pathState;
    }
    private List<LynxModule> allHubs;
    @Override
    public void init() {
        intakeController=new IntakeController(hardwareMap);
        turretController=new TurretController(hardwareMap);
        shooterControllerjana=new ShooterControllerjana(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        stateController=new StateControllerAuto(hardwareMap,shooterControllerjana,intakeController,turretController,follower,true);
        turretController.setTargetPosition(180);
        buildPaths();
        follower.setStartingPose(startPose);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

    }
    @Override
    public void init_loop() {
        turretController.update();
    }
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
        stateController.update();
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