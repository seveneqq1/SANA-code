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
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterControllerjana;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateControllerAuto;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(group = "!competition")
public class Red_Near_without_3rd_line extends OpMode {
    private Timer opmodeTimer;
    private Follower follower;
    IntakeController intakeController;
    ShooterControllerjana shooterControllerjana;
    TurretController turretController;
    StateControllerAuto stateController;
    private final Pose score_pose_for_preload=new Pose(108,108,Math.toRadians(270));
    private PathChain scorePreload, intake_first_line, score_first_line, intake_second, score_second, intake_gate,score_gate, intake_gate_2,score_gate_2, intake_gate_3,score_gate_3,intake_gate_4,score_gate_4,park;
    public enum Path_States{
        ZERO,
        SCORE_PRELOAD,
        INTAKE_FIRST,SCORE_FIRST,
        INTAKE_SECOND, SCORE_SECOND,
        INTAKE_GATE,SCORE_GATE,WAIT_FOR_THE_GATE,
        INTAKE_GATE_2,SCORE_GATE_2,WAIT_FOR_THE_GATE_2,
        INTAKE_GATE_3,SCORE_GATE_3,WAIT_FOR_THE_GATE_3,

        INTAKE_GATE_4,SCORE_GATE_4,WAIT_FOR_THE_GATE_4,
        PARK,STOP,
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
                            setPathState(Path_States.INTAKE_GATE_4);
                        }
                        if(prevState==7 && is_shoot_ended()){
                            stateController.intaking();
                            setPathState(Path_States.PARK);
                        }
                    }
                }
                    break;
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                stateController.offset_for_tracking=7;
                follower.setMaxPower(0.7);
                setPathState(Path_States.INTAKE_SECOND);
                break;
            case INTAKE_SECOND:
                if(follower.getCurrentTValue()>0.3){
                    stateController.shoot_whereever_you_want_auto();
                }
                if(follower.getCurrentTValue()>0.97 || is_shoot_ended()) {
                    follower.followPath(intake_second);
                    stateController.offset_for_tracking=8 ;
                    stateController.is_it_shoot_while_move=false;
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.SCORE_SECOND);
                }
                break;
            case SCORE_SECOND:
                if(follower.getCurrentTValue()>0.98){
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
                if(!follower.atPose(Pose_states.open_gate_red,4,4)){
                timer.reset();
            }
            if(follower.atPose(Pose_states.open_gate_red,5,5)){
                setPathState(Path_States.SCORE_GATE);
            }
                break;
            case SCORE_GATE:
                if(timer.milliseconds()>1300){
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
                if(follower.getPose().getX()>115){
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
                if(!follower.atPose(Pose_states.open_gate_red,4,4)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_red,5,5)){
                    setPathState(Path_States.SCORE_GATE_2);
                }
                break;
            case SCORE_GATE_2:
                if( timer.milliseconds()>1650){
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
                if(!follower.atPose(Pose_states.open_gate_red,6,6)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_red,6,6)){
                    setPathState(Path_States.SCORE_GATE_3);
                }
                break;
            case SCORE_GATE_3:
                if(timer.milliseconds()>1450){
                    follower.followPath(score_gate_3);
                    prevState=6;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case INTAKE_GATE_4:
                follower.followPath(intake_gate_4);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_4);
                break;
            case WAIT_FOR_THE_GATE_4:
                if(!follower.atPose(Pose_states.open_gate_red,8,8)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_red,8,8)){
                    setPathState(Path_States.SCORE_GATE_4);
                }
                break;
            case SCORE_GATE_4:
                if(timer.milliseconds()>1450){
                    follower.followPath(score_gate_4);
                    prevState=7;
                    setPathState(Path_States.SHOOT);
                }
                break;
            case PARK:
                follower.followPath(park);
                follower.setMaxPower(1);
                setPathState(Path_States.STOP);
                break;
            case STOP:
                if(follower.isRobotStuck()){
                    requestOpModeStop();
                }
                break;
        }
    }
     
    public void setPathState(Path_States pathState) {
        this.pathStates=pathState;
    }
    private List<LynxModule> allHubs;
    GamepadEx driver;
    VoltageSensor voltageSensor;
    @Override
    public void init() {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        driver=new GamepadEx(gamepad1);
        timer=new ElapsedTime();
        intakeController=new IntakeController(hardwareMap);
        turretController=new TurretController(hardwareMap);
        shooterControllerjana=new ShooterControllerjana(hardwareMap,0.03,0);
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

        stateController.offset_velocity=-60;
        stateController.hood_comp_multiply=1.4;
        opmodeTimer.resetTimer();
        setPathState(Path_States.SCORE_PRELOAD);
    }
    private double lastTimeStamp=0;
     
     
    @Override
    public void loop() {
        driver.readButtons();
        if(driver.wasJustPressed(GamepadKeys.Button.X)){
            stateController.offset_for_tracking+=2;
        }
        if(driver.wasJustPressed(GamepadKeys.Button.B)){
            stateController.offset_for_tracking-=2;
        }
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();
        stateController.update();
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0)
            lastTimeStamp = currentTimeStamp;
        double dt = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;
        double frequency = 1 / dt;
        telemetry.addData("voltage",voltageSensor.getVoltage());
        telemetry.addData("state",pathStates);
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