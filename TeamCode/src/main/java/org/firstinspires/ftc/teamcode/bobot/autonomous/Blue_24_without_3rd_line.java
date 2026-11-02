package org.firstinspires.ftc.teamcode.bobot.autonomous; 
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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

@Autonomous(group = "!competition")
public class Blue_24_without_3rd_line extends OpMode {
    private Timer opmodeTimer;
    private Follower follower;
    IntakeController intakeController;
    ShooterControllerjana shooterControllerjana;
    TurretController turretController;
    StateControllerAuto stateController;
    private final Pose score_pose_for_preload=new Pose(108,108,Math.toRadians(270));
    private PathChain scorePreload, intake_first_line, score_first_line, intake_second, score_second, intake_gate,score_gate, intake_gate_2,score_gate_2, intake_gate_3,score_gate_3,intake_third_line,score_third_line,intake_gate_4,score_gate_4,intake_gate_5,score_gate_5,park;
    public enum Path_States{
        ZERO,
        SCORE_PRELOAD,
        INTAKE_FIRST,SCORE_FIRST,SHOOT_FIRST_LINE,SHOOT_FIRST_LINE_2,
        INTAKE_SECOND, SCORE_SECOND,SHOOT_SECOND_LINE,SHOOT_SECOND_LINE_2,
        INTAKE_GATE,SCORE_GATE,WAIT_FOR_THE_GATE, SHOOT_GATE,SHOOT_GATE_2,
        INTAKE_GATE_2,SCORE_GATE_2,WAIT_FOR_THE_GATE_2,SECOND_SHOOT_GATE,SECOND_SHOOT_GATE_2,
        INTAKE_GATE_3,SCORE_GATE_3,WAIT_FOR_THE_GATE_3,THIRD_SHOOT_GATE,THIRD_SHOOT_GATE_2,

        INTAKE_GATE_4,SCORE_GATE_4,WAIT_FOR_THE_GATE_4,FOURTH_SHOOT_GATE,FOURTH_SHOOT_GATE_2,

        INTAKE_GATE_5,SCORE_GATE_5,WAIT_FOR_THE_GATE_5,FIFTH_SHOOT_GATE,FIFTH_SHOOT_GATE_2,
        INTAKE_THIRD,SCORE_THIRD,
        PARK,STOP,
        SHOOT;
    }
    private Path_States pathStates= Path_States.ZERO;
    public double reverse(double x){
        return (144-x);
    }
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.5, 113.000),
                               new Pose(34,116),
                               new Pose(54,86)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intake_first_line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54, 86.000),
                                new Pose(20, 89.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        score_first_line = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24, 89.000),
                               Pose_states.score_pose_for_blue
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake_second = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54,86),
                                new Pose(59, 60),
                                new Pose(23, 64)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        score_second = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23, 64),
                                Pose_states.score_pose_for_blue
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake_gate=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.score_pose_for_blue,
                                new Pose(reverse(88.232), 52.240),
                                new Pose(reverse(106.363), 63.849),
                                new Pose(Pose_states.open_gate_blue.getX()-2,Pose_states.open_gate_blue.getY()+1.5)                        )
                )
                .setLinearHeadingInterpolation(Pose_states.score_pose_for_blue.getHeading(),Pose_states.open_gate_blue.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate=follower.pathBuilder()
                .addPath(
 
 
 
 
 
                        new BezierLine(
                                new Pose(Pose_states.open_gate_blue.getX(),Pose_states.open_gate_blue.getY()+1),
 
                                Pose_states.score_pose_for_blue
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake_gate_2=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.score_pose_for_blue,
                                new Pose(reverse(88.232), 52.240),
                                new Pose(reverse(106.363), 63.849),
                                new Pose(Pose_states.open_gate_blue.getX()-3,Pose_states.open_gate_blue.getY()+1.5)                        )
                )
                .setLinearHeadingInterpolation(Pose_states.score_pose_for_blue.getHeading(),Pose_states.open_gate_blue.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate_2=follower.pathBuilder()
                .addPath(
 
 
 
 
 
                        new BezierLine(
                                new Pose(Pose_states.open_gate_blue.getX(),Pose_states.open_gate_blue.getY()+1),
 
                                Pose_states.score_pose_for_blue
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake_gate_3=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.score_pose_for_blue,
                                new Pose(reverse(88.232), 52.240),
                                new Pose(reverse(106.363), 63.849),
                                new Pose(Pose_states.open_gate_blue.getX()-4,Pose_states.open_gate_blue.getY()+1.5)                        )
                )
                .setLinearHeadingInterpolation(Pose_states.score_pose_for_blue.getHeading(),Pose_states.open_gate_blue.getHeading())
                .setTimeoutConstraint(800)

                .build();
        score_gate_3=follower.pathBuilder()
                .addPath(
 
 
 
 
 
                        new BezierLine(
                                new Pose(Pose_states.open_gate_blue.getX(),Pose_states.open_gate_blue.getY()+1),
 
                                Pose_states.score_pose_for_blue
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake_gate_4=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.score_pose_for_blue,
                                new Pose(reverse(88.232), 52.240),
                                new Pose(reverse(106.363), 63.849),
                                new Pose(Pose_states.open_gate_blue.getX()-4.5,Pose_states.open_gate_blue.getY()+1.5)                        )
                )
                .setLinearHeadingInterpolation(Pose_states.score_pose_for_blue.getHeading(),Pose_states.open_gate_blue.getHeading())
                .setTimeoutConstraint(800)

                .build();
        score_gate_4=follower.pathBuilder()
                .addPath(
 
 
 
 
 
                        new BezierLine(
                                new Pose(Pose_states.open_gate_blue.getX(),Pose_states.open_gate_blue.getY()+1),
 
                                Pose_states.score_pose_for_blue
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake_gate_4=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.score_pose_for_blue,
                                new Pose(reverse(88.232), 52.240),
                                new Pose(reverse(106.363), 63.849),
                                new Pose(Pose_states.open_gate_blue.getX()-5,Pose_states.open_gate_blue.getY()+2)                        )
                )
                .setLinearHeadingInterpolation(Pose_states.score_pose_for_blue.getHeading(),Pose_states.open_gate_blue.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate_4=follower.pathBuilder()
                .addPath(
 
 
 
 
 
                        new BezierLine(
                                new Pose(Pose_states.open_gate_blue.getX(),Pose_states.open_gate_blue.getY()+1),
 
                                Pose_states.score_pose_for_blue
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTimeoutConstraint(0)
                .build();
        intake_gate_5=follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                Pose_states.score_pose_for_blue,
                                new Pose(reverse(88.232), 52.240),
                                new Pose(reverse(106.363), 63.849),
                                new Pose(Pose_states.open_gate_blue.getX()-5,Pose_states.open_gate_blue.getY()+2.5)                        )
                )
                .setLinearHeadingInterpolation(Pose_states.score_pose_for_blue.getHeading(),Pose_states.open_gate_blue.getHeading())
                .setTimeoutConstraint(800)
                .build();
        score_gate_5=follower.pathBuilder()
                .addPath(
 
 
 
 
 
                        new BezierCurve(
                                new Pose(Pose_states.open_gate_blue.getX(),Pose_states.open_gate_blue.getY()+1),
new Pose(58,62),
 
                                new Pose(58,117)
                        )
                )
 
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTimeoutConstraint(0)
                .build();
        park=follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                Pose_states.score_pose_for_blue,
                                Pose_states.startPose_Blue_TeleOp
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                stateController.offset_for_tracking=-5;
                follower.setMaxPower(1);
                setPathState(Path_States.INTAKE_SECOND);
                break;
            case INTAKE_SECOND:
                if(follower.getCurrentTValue()>0.3){
                    stateController.shoot_whereever_you_want_auto();
                }
                if(follower.getCurrentTValue()>0.97 || is_shoot_ended()) {
                    follower.followPath(intake_second);
                    stateController.offset_for_tracking=-6;
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
                    setPathState(Path_States.SHOOT_SECOND_LINE);
                }
                break;
            case SHOOT_SECOND_LINE:
                if(follower.atPose(Pose_states.score_pose_for_blue,4,4)){
                    stateController.shoot_whereever_you_want_auto();
                    follower.setMaxPower(0.4);
                    setPathState(Path_States.SHOOT_SECOND_LINE_2);
                }
                break;
            case SHOOT_SECOND_LINE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE);
                }
                break;
            case INTAKE_GATE:
                follower.followPath(intake_gate);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE);
                break;
            case WAIT_FOR_THE_GATE:
                if(!follower.atPose(Pose_states.open_gate_blue,4,4)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_blue,5,5)){
                    setPathState(Path_States.SCORE_GATE);
                }
                break;
            case SCORE_GATE:
                if( timer.milliseconds()>1000){
                    follower.followPath(score_gate);
                    prevState=3;
                    setPathState(Path_States.SHOOT_GATE);
                }
                break;
            case SHOOT_GATE:
                if(follower.atPose(Pose_states.score_pose_for_blue,5,5)){
                    shoot();
                    setPathState(Path_States.SHOOT_GATE_2);
                }
                break;
            case SHOOT_GATE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_FIRST);
                }
                break;
            case INTAKE_FIRST:
                follower.followPath(intake_first_line);
                follower.setMaxPower(1);
                setPathState(Path_States.SCORE_FIRST);
                break;
            case SCORE_FIRST:
                if(follower.getCurrentTValue()>0.5 && follower.getPose().getX()<25){
                    follower.followPath(score_first_line);
                    follower.setMaxPower(1);
                    setPathState(Path_States.SHOOT_FIRST_LINE);
                }
                break;
            case SHOOT_FIRST_LINE:
                if(follower.atPose(Pose_states.score_pose_for_blue,4,4)){
                    shoot();
                    setPathState(Path_States.SHOOT_FIRST_LINE_2);
                }
                break;
            case SHOOT_FIRST_LINE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    follower.setMaxPower(1);
                    stateController.intaking();
                    setPathState(Path_States.INTAKE_GATE_2);
                }
                break;
            case INTAKE_GATE_2:
                follower.followPath(intake_gate_2);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_2);
                break;
            case WAIT_FOR_THE_GATE_2:
                if(!follower.atPose(Pose_states.open_gate_blue,6,4)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_blue,6,5)){
                    setPathState(Path_States.SCORE_GATE_2);
                }
                break;
            case SCORE_GATE_2:
                if( timer.milliseconds()>1200){
                    follower.followPath(score_gate_2);
                    prevState=5;
                    setPathState(Path_States.SECOND_SHOOT_GATE);
                }
                break;
            case SECOND_SHOOT_GATE:
                if(follower.atPose(Pose_states.score_pose_for_blue,6,6)){
                    shoot();
                    setPathState(Path_States.SECOND_SHOOT_GATE_2);
                }
                break;
            case SECOND_SHOOT_GATE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE_3);
                }
                break;
            case INTAKE_GATE_3:
                follower.followPath(intake_gate_3);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_3);
                break;
            case WAIT_FOR_THE_GATE_3:
                if(!follower.atPose(Pose_states.open_gate_blue,7,4)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_blue,7,5)){
                    setPathState(Path_States.SCORE_GATE_3);
                }
                break;
            case SCORE_GATE_3:
                if( timer.milliseconds()>1500){
                    follower.followPath(score_gate_3);
                    prevState=3;
                    setPathState(Path_States.THIRD_SHOOT_GATE);
                }
                break;
            case THIRD_SHOOT_GATE:
                if(follower.atPose(Pose_states.score_pose_for_blue,6,6)){
                    shoot();
                    setPathState(Path_States.THIRD_SHOOT_GATE_2);
                }
                break;
            case THIRD_SHOOT_GATE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE_4);
                }
                break;
            case INTAKE_GATE_4:
                follower.followPath(intake_gate_4);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_4);
                break;
            case WAIT_FOR_THE_GATE_4:
                if(!follower.atPose(Pose_states.open_gate_blue,8,4)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_blue,8,5)){
                    setPathState(Path_States.SCORE_GATE_4);
                }
                break;
            case SCORE_GATE_4:
                if( timer.milliseconds()>1500){
                    follower.followPath(score_gate_4);
                    prevState=3;
                    setPathState(Path_States.FOURTH_SHOOT_GATE);
                }
                break;
            case FOURTH_SHOOT_GATE:
                if(follower.atPose(Pose_states.score_pose_for_blue,6,6)){
                    shoot();
                    setPathState(Path_States.FOURTH_SHOOT_GATE_2);
                }
                break;
            case FOURTH_SHOOT_GATE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.INTAKE_GATE_5);
                }
                break;
            case INTAKE_GATE_5:
                follower.followPath(intake_gate_5);
                follower.setMaxPower(1);
                setPathState(Path_States.WAIT_FOR_THE_GATE_5);
                break;
            case WAIT_FOR_THE_GATE_5:
                if(!follower.atPose(Pose_states.open_gate_blue,8,4)){
                    timer.reset();
                }
                if(follower.atPose(Pose_states.open_gate_blue,8,5)){
                    setPathState(Path_States.SCORE_GATE_5);
                }
                break;
            case SCORE_GATE_5:
                if( timer.milliseconds()>1500){
                    follower.followPath(score_gate_5);
                    stateController.is_it_shoot_while_move=true;
                    prevState=3;
                    setPathState(Path_States.FIFTH_SHOOT_GATE);
                }
                break;
            case FIFTH_SHOOT_GATE:
                if(follower.getCurrentTValue()>0.6){
                    shoot();
                    setPathState(Path_States.FIFTH_SHOOT_GATE_2);
                }
                break;
            case FIFTH_SHOOT_GATE_2:
                if(is_shoot_ended() && !follower.isBusy()){
                    stateController.intaking();
                    follower.setMaxPower(1);
                    setPathState(Path_States.PARK);
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
    ElapsedTime time;
    @Override
    public void init() {
        time=new ElapsedTime();
        timer=new ElapsedTime();
        intakeController=new IntakeController(hardwareMap);
        turretController=new TurretController(hardwareMap);
        shooterControllerjana=new ShooterControllerjana(hardwareMap,0.035,0);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setDrivePIDFCoefficients(
                new FilteredPIDFCoefficients(0.0095, 0, 0.0017, 0.6, 0));
        stateController=new StateControllerAuto(hardwareMap,shooterControllerjana,intakeController,turretController,follower,false);
        turretController.setTargetPosition(180);
        buildPaths();
        follower.setStartingPose(new Pose(18.5,113.1,Math.toRadians(180)));
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

    }
    @Override
    public void init_loop() {
        turretController.setTargetPosition(315);
        turretController.update();
    }
    @Override
    public void start() {
        stateController.offset_velocity=-20;
        stateController.hood_comp_multiply=1.2;
        time.reset();
        opmodeTimer.resetTimer();
        setPathState(Path_States.SCORE_PRELOAD);
    }
    private double lastTimeStamp=0;
     
    @Override
    public void loop() {
        if(time.milliseconds()>31500){
            requestOpModeStop();
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