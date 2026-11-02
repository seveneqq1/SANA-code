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
import org.opencv.core.Mat;

import java.util.List;

@Autonomous(group = "!competition")
public class Red_Near extends OpMode {
    private Timer opmodeTimer;
    private Follower follower;
    IntakeController intakeController;
    ShooterControllerjana shooterControllerjana;
    TurretController turretController;
    StateControllerAuto stateController;
    private final Pose score_pose_for_preload=new Pose(108,108,Math.toRadians(270));
    private PathChain scorePreload, intake_first_line, score_first_line, intake_second, score_second, intake_gate,score_gate, intake_gate_2,score_gate_2, intake_gate_3,score_gate_3,intake_third_line,score_third_line;
    public enum Path_States{
        ZERO,
        SCORE_PRELOAD,
        INTAKE_FIRST,SCORE_FIRST,
        INTAKE_SECOND, SCORE_SECOND,
        INTAKE_GATE,SCORE_GATE,WAIT_FOR_THE_GATE,
        INTAKE_GATE_2,SCORE_GATE_2,WAIT_FOR_THE_GATE_2,
        INTAKE_GATE_3,SCORE_GATE_3,WAIT_FOR_THE_GATE_3,
        INTAKE_THIRD,SCORE_THIRD,
        SHOOT;
    }
    private Path_States pathStates= Path_States.ZERO;
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
                                new Pose(111.000, 85.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score_first_line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(111.000, 85.000),
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
                                new Pose(80.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(0,Math.toRadians(331))
                .build();
        intake_gate=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(80,75),
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
                                new Pose(80,75)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        intake_gate_2=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(80,75),
                                new Pose(88.232, 52.240),
                                new Pose(106.363, 63.849),
                                new Pose(Pose_states.open_gate_red.getX(),Pose_states.open_gate_red.getY())
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
                                new Pose(80,75)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        intake_gate_3=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(80,75),
                                new Pose(88.232, 52.240),
                                new Pose(106.363, 63.849),
                                new Pose(Pose_states.open_gate_red.getX(),Pose_states.open_gate_red.getY())
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
                                new Pose(80,75)
                        )
                )
                .setLinearHeadingInterpolation(Pose_states.open_gate_red.getHeading(),Math.toRadians(331))
                .build();
        intake_third_line=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(80,75),
                                new Pose(76,31),
                                new Pose(111,33)
                        )
                )
                .setConstantHeadingInterpolation(0)
                .build();
        score_third_line=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(111,33),
                                new Pose(94,118)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTValueConstraint(0.9)
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
    ElapsedTime timer;
    public void autonomousPathUpdate() {
        switch (pathStates) {
            case SHOOT:
                if(follower.getCurrentTValue()>0.5 && prevState==7){
                    shoot();
                }
                if(!follower.isBusy()){
                    shoot();
                    if(prevState==1 || prevState==2 || prevState==3 || prevState==4 || prevState==5 || prevState ==6 || prevState==7){
                        if(prevState==1 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_SECOND);
                        }
                        if(prevState==2 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_GATE);
                        }
                        if(prevState==3 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_FIRST);
                        }
                        if(prevState==4 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_GATE_2);
                        }
                        if(prevState==5 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_GATE_3);
                        }
                        if(prevState==6 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.INTAKE_THIRD);
                        }
                        if(prevState==7 && is_shoot_ended()){
                            requestOpModeStop();
                        }
                    }
                }
                    break;
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                stateController.offset_for_tracking=4;
                follower.setMaxPower(0.6);
                setPathState(Path_States.INTAKE_SECOND);
                break;
            case INTAKE_SECOND:
                if(follower.getCurrentTValue()>0.45){
                    stateController.shoot_whereever_you_want_auto();
                }
                if(follower.getCurrentTValue()>0.97 || is_shoot_ended()) {
                    follower.followPath(intake_second);
                    stateController.offset_for_tracking=6;
                    stateController.is_it_shoot_while_move=false;
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.SCORE_SECOND);
                }
                break;
            case SCORE_SECOND:
                if(follower.getCurrentTValue()>0.99){
                    follower.followPath(score_second);
                    follower.setMaxPower(1);
                    prevState=2;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_GATE:
                follower.followPath(intake_gate);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE);
                break;
            case WAIT_FOR_THE_GATE:
                if(follower.isBusy()){
                    timer.reset();
                }
                if(!follower.isBusy()){
                    setPathState(Path_States.SCORE_GATE);
                }
                break;
            case SCORE_GATE:
                if(!follower.isBusy() && timer.milliseconds()>1300){
                    follower.followPath(score_gate);
                    prevState=3;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_FIRST:
                follower.followPath(intake_first_line);
                follower.setMaxPower(1);
                setPathState(Path_States.SCORE_FIRST);
                break;
            case SCORE_FIRST:
                if(follower.getCurrentTValue()>0.99){
                    follower.followPath(score_first_line);
                    follower.setMaxPower(1);
                    prevState=4;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_GATE_2:
                follower.followPath(intake_gate_2);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_2);
                break;
            case WAIT_FOR_THE_GATE_2:
                if(follower.isBusy()){
                    timer.reset();
                }
                if(!follower.isBusy()){
                    setPathState(Path_States.SCORE_GATE_2);
                }
                break;
            case SCORE_GATE_2:
                if(!follower.isBusy() && timer.milliseconds()>1300){
                    follower.followPath(score_gate_2);
                    prevState=5;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_GATE_3:
                follower.followPath(intake_gate_3);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_3);
                break;
            case WAIT_FOR_THE_GATE_3:
                if(follower.isBusy()){
                    timer.reset();
                }
                if(!follower.isBusy()){
                    setPathState(Path_States.SCORE_GATE_3);
                }
                break;
            case SCORE_GATE_3:
                if(!follower.isBusy() && timer.milliseconds()>1300){
                    follower.followPath(score_gate_3);
                    prevState=6;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_THIRD:
                follower.followPath(intake_third_line);
                stateController.offset_for_tracking=0;
                StateControllerAuto.use_hypotenus=true;
                stateController.is_it_shoot_while_move=true;
                follower.setMaxPower(1);
                setPathState(Path_States.SCORE_THIRD);
                break;
            case SCORE_THIRD:
                if(follower.getCurrentTValue()>0.98){
                    follower.followPath(score_third_line);
                    follower.setMaxPower(1);
                    prevState=7;
                    setPathState(Path_States.SHOOT);
                }
                break;
        }
    }
     
    public void setPathState(Path_States pathState) {
        this.pathStates=pathState;
    }
    private List<LynxModule> allHubs;
    @Override
    public void init() {
        timer=new ElapsedTime();
        intakeController=new IntakeController(hardwareMap);
        turretController=new TurretController(hardwareMap);
        shooterControllerjana=new ShooterControllerjana(hardwareMap);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        stateController=new StateControllerAuto(hardwareMap,shooterControllerjana,intakeController,turretController,follower,true);
        turretController.setTargetPosition(180);
        buildPaths();
        follower.setStartingPose(new Pose(121,111,0));
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

    }
    @Override
    public void init_loop() {
        turretController.setTargetPosition(45);
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