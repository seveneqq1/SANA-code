package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.bobot.states.ModeStates;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.bobot.utlities.MyMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class StateController {
    Limelight3A limelight;

    public ElapsedTime timer;
    public ElapsedTime deviate_timer;
    Follower follower;
    ShooterControllerjana shooterController;
    IntakeController intakeController;
    TurretController turretController;
    GoBildaPinpointDriver pinpoint;
    public boolean isShooterAlwayson=true;
    public boolean use_only_heading_for_tracking=false;
    public static double hood_offset_multiplier=1  ;
    public static double blue_offset_for_tracking_with_only_heading=20;
    public static double offset_for_tracking=-2;
    public static double blue_offset_for_tracking=14;
    public static double tangent_offset=35;
    private State state = State.ZERO;
    public enum State{
        ZERO,
        INTAKING_1,INTAKING_2,INTAKING_3,
        SHOOT_NEAR,SHOOT_NEAR_2,
        JUST_PUSH_BALLS_TO_TRANSFER, JUST_PUSH_BALLS_TO_TRANSFER_AUTO,
        SHOOT_WHERE_EVER,SHOOT_WHERE_EVER_2,
        SHOOT_WHERE_EVER_AUTO,SHOOT_WHERE_EVER_2_AUTO,SHOOT_ENDED_AUTO,
        RESET_BLUE,RESET_RED,RESET_RED_GOAL,RESET_NEAR_BLUE_GOAL,RESET_RED_GATE,RESET_BLUE_GATE

    }
    public State getState() {
        return state;
    }
    public boolean auto;
    private boolean redd;
    Servo rgb_left,rgb_right;

    public StateController(HardwareMap hardwareMap, ShooterControllerjana shooterController, IntakeController intakeController, TurretController turretController, Follower follower, Pose startpose, boolean red) {
        this.shooterController=shooterController;
        this.intakeController=intakeController;
        this.turretController=turretController;
        this.follower=follower;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startpose);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        timer=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        limelight =hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        rgb_left =hardwareMap.get(Servo.class,"rgb");
        rgb_right=hardwareMap.get(Servo.class,"rgb2");
        deviate_timer=new ElapsedTime();
        shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
        ModeStates.isRedAlliance=red;
        auto=false;
        redd=red;
        if(red){
            goal= Pose_states.red_goal_near;
        }
        if(!red){
            goal=Pose_states.blue_goal_near;
        }

    }

    public boolean shooting=false;
    public double hypotenus;
    double x_inches_to_the_goal,y_inches_to_the_goal;
    public static double arc_tan_to_the_red_goal;
    public double turret_target;
    private static double currentHeading = 0;
    private double currentRaw = 0;
    Pose goal;
    double atan2;
    double x_velocity,y_velocity;
    double hood_compensation;
    double current_X,current_Y;
    double predicted_x,predicted_y;
    Pose predictedPose;
    public boolean far_zone;
    public boolean is_it_near_zones;
    public void start(){
        turretController.setTargetPosition(TurretController.center_target_position);
        turretController.update();
    }
    public boolean goal_aiming=false;
public static double hood_degrees_near_zone(double x){
    return MathFunctions.clamp(-0.0000525015*x*x*x+0.0119869*x*x-0.6101*x+39.43415,31,47);
}
    public static double shooter_speed_near_zone(double x){
        return  MathFunctions.clamp(0.000769272*x*x+6.82352*x+621.70813,925,1360);
    }
    public static double hood_degrees_far_zone(double x){
        return 48;
    }
    public static double shooter_speed_far_zone(double x){
        return  MathFunctions.clamp(0.001343696*x*x*x*x-0.7908479*x*x*x+174.33431*x*x-17052.3518*x+625618.891,1325,1450);
    }
    public static double hood_comp(double x){
    return MathFunctions.clamp(-0.00870721*x*x-3.95664*x+1146.26336,625,900);
    }
    public static double hood_comp_far_zone(double x){
        return MathFunctions.clamp(-0.00871963*x*x*x+3.66656*x*x-505.07665*x+23168.6747,330,500);
    }


    public static double ball_shoot_time(double dist){
        return MathFunctions.clamp(dist*11,100,1200);
    }

    public void isShooterAlwaysOn(){
        isShooterAlwayson=!isShooterAlwayson;
    }
    public void setVioletColour(){
        rgb_left.setPosition(0.66);
        rgb_right.setPosition(0.66);
    }
    public void setRedColour(){
        rgb_right.setPosition(0.28);
        rgb_left.setPosition(0.28);
    }
    public void cancel_everything(){
        intakeController.setState(IntakeController.State.ZERO);
        shooterController.setShooterVelocity(0);
        shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
        isShooterAlwayson=false;
        shooting=false;
        setState(State.ZERO);
    }
    public void intaking(){
        intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
        shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
        open_yogurt=false;
        shooting=false;
        setVioletColour();
        setState(State.INTAKING_1);
    }
    public void balls_out(){
    intakeController.setState(IntakeController.State.BALLS_OUT);
    setState(State.ZERO);
    }
    public void move_turret(double degrees){
    offset_for_tracking+=degrees;
    }
    public void turret_state_change(){
    goal_aiming=!goal_aiming;
    }
    public void shoot_whereever_you_want(){
        if(state==State.ZERO|| state==State.INTAKING_2 || state==State.INTAKING_1){
            setRedColour();
            timer.reset();
            shooting=true;
            timer.reset();
            setState(State.SHOOT_WHERE_EVER);
        }
    }
    public void shoot_whereever_you_want_auto(){
        if(state==State.ZERO|| state==State.INTAKING_2 ){
            timer.reset();
            shooting=true;
            intakeController.setState(IntakeController.State.SLIGHTLY_PUSH_BALLS_OUT);
            timer.reset();
            setState(State.JUST_PUSH_BALLS_TO_TRANSFER_AUTO);
        }
    }
    public void reset_red_human(){
        pinpoint.resetPosAndIMU();
        setState(State.RESET_RED);
    }
    public void reset_blue_human(){
        pinpoint.resetPosAndIMU();
        setState(State.RESET_BLUE);
    }
    public void reset_red_gate(){
        pinpoint.resetPosAndIMU();
        setState(State.RESET_RED_GATE);
    }
    public void reset_blue_gate(){
        pinpoint.resetPosAndIMU();
        setState(State.RESET_BLUE_GATE);
    }
    public void reset_red_goal(){
    pinpoint.resetPosAndIMU();
    setState(State.RESET_RED_GOAL);
    }
    public void reset_near_blue_goal(){
    pinpoint.resetPosAndIMU();
    setState(State.RESET_NEAR_BLUE_GOAL);
    }
    public void change_turret_tracking(){
        use_only_heading_for_tracking=!use_only_heading_for_tracking;
    }
    public void shoot_manual_near(){
    hypotenus_for_manual_control=54;
    shoot_whereever_you_want();
    }
    public void shoot_manual_middle(){
    hypotenus_for_manual_control=71.2;
        shoot_whereever_you_want();
    }
    public void shoot_manual_far_middle(){
        hypotenus_for_manual_control=84;
        shoot_whereever_you_want();
    }
    public void switch_to_manual_mode(){
    if(!switch_to_manual_mode){
        use_only_heading_for_tracking=true;
    }
        if(switch_to_manual_mode){
            use_only_heading_for_tracking=false;
        }
    switch_to_manual_mode=!switch_to_manual_mode;
    }
    public boolean is_it_shoot_while_move=false;
    public void shoot_while_move_state_change(){
        is_it_shoot_while_move=!is_it_shoot_while_move;
    }
    public double getHeading() {
        return currentHeading;
    }
    public static boolean open_yogurt=false;
    public static double offset_for_Velocity=0;
    public boolean switch_to_manual_mode=false;
    public static double hypotenus_for_manual_control=70;
    public void update(){
        LLResult result = limelight.getLatestResult();
        if(result != null && result.isValid() && !result.getFiducialResults().isEmpty()){
            Pose3D npose = result.getBotpose();
            if(npose != null){
                double xIn = npose.getPosition().x * 39.3701;
                double yIn = npose.getPosition().y * 39.3701;
                double hRad = npose.getOrientation().getYaw(AngleUnit.RADIANS);
                double robotVel = Math.hypot(x_velocity, y_velocity);
                if(robotVel < 2){
                    follower.setPose(new Pose(xIn, yIn, hRad));
                }
            }
        }
        if(shooting || isShooterAlwayson){
            if(!far_zone){
                double compensation=(shooterController.getTargetPositon()-shooterController.getCurrentVelocity());
                hood_compensation=(compensation)/hood_comp(hypotenus)*hood_offset_multiplier;
            shooterController.setShooterVelocity(shooter_speed_near_zone(hypotenus)+offset_for_Velocity);
 
                shooterController.setRampPosition(hood_degrees_near_zone(hypotenus)-hood_compensation);
                    if(hood_compensation<0){
                        hood_compensation=0;
                    }
            }
            if(far_zone){
                double compensation=(shooterController.getTargetPositon()-shooterController.currentVelocity);
                hood_compensation=(compensation)/hood_comp_far_zone(hypotenus)*hood_offset_multiplier*1.2;
                shooterController.setShooterVelocity(shooter_speed_far_zone(hypotenus));
                if(!shooting) {
                    shooterController.setRampPosition(hood_degrees_far_zone(hypotenus));
                }
                if(shooting && hood_compensation>=0){
                    shooterController.setRampPosition(hood_degrees_far_zone(hypotenus)-hood_compensation);
                }
            }
        }

        currentRaw = follower.getPose().getHeading() /6.28 * 360.0;
        if(currentRaw<0){
            currentRaw=360+currentRaw;
        }  
        currentHeading=currentRaw;

        x_velocity=follower.getVelocity().getXComponent();
        y_velocity=follower.getVelocity().getYComponent();

        predicted_x=x_velocity*(ball_shoot_time(hypotenus)/1000);
        predicted_y=y_velocity*(ball_shoot_time(hypotenus)/1000);

        predictedPose =new Pose(follower.getPose().getX()+predicted_x,follower.getPose().getY()+predicted_y,currentRaw);
        if(!is_it_shoot_while_move) {
            current_X = follower.getPose().getX();
            current_Y = follower.getPose().getY();
        }
        if(is_it_shoot_while_move){
            current_X=predictedPose.getX();
            current_Y=predictedPose.getY();
        }
        if(((Math.abs(current_X-72)+42)<=current_Y) || (-(Math.abs(current_X-72)-40))>=current_Y){
            is_it_near_zones=true;
        }
        else{
            is_it_near_zones=false;
        }
        far_zone= follower.getPose().getY() < 56;
        if(redd){
            if(far_zone){
                goal=Pose_states.red_goal_far;
            }
            if(!far_zone){
                goal=Pose_states.red_goal_near;
            }
        }

        if(!redd){
            if(far_zone){
                goal=Pose_states.blue_goal_far;
            }
            if(!far_zone){
                goal=Pose_states.blue_goal_near;
            }
        }

        x_inches_to_the_goal=Math.abs(goal.getX()-current_X);
        y_inches_to_the_goal=goal.getY()-current_Y;

        arc_tan_to_the_red_goal= Math.abs(MyMath.arcTan(y_inches_to_the_goal/x_inches_to_the_goal));
        if(!switch_to_manual_mode) {
            hypotenus = MyMath.pifagor(x_inches_to_the_goal, y_inches_to_the_goal);
        }
        if(switch_to_manual_mode){
            hypotenus=hypotenus_for_manual_control;
        }
        if(redd){
            if(!use_only_heading_for_tracking) {
                if (currentRaw > 0 && currentRaw < 35) {
                    turret_target = Math.abs(currentRaw - arc_tan_to_the_red_goal) +offset_for_tracking+3;
                } else {
                    turret_target = 360 - (currentRaw - arc_tan_to_the_red_goal) +offset_for_tracking+3;
                }
            }
            if(use_only_heading_for_tracking){
                if (currentRaw > 0 && currentRaw < 35) {
                    turret_target = Math.abs(currentRaw - tangent_offset) +offset_for_tracking+3;
                } else {
                    turret_target = 360 - (currentRaw - tangent_offset) +offset_for_tracking+3;
                }
            }
        }
        if(!redd){
            if(!use_only_heading_for_tracking) {
 
 
 
 
  
                    turret_target = 180-currentRaw - arc_tan_to_the_red_goal+offset_for_tracking+4;
                    turret_target=turret_target%360;
                    if(turret_target<0){
                        turret_target+=360;
                    }
                    if(far_zone){
                        turret_target-=2;
                    }

            }
            if(use_only_heading_for_tracking){
                turret_target = 180-currentRaw - tangent_offset+offset_for_tracking+2;
                turret_target=turret_target%360;
                if(turret_target<0){
                    turret_target+=360;
                }
                if(far_zone){
                    turret_target-=2;
                }
            }
        }
        if(turret_target<0){
            turret_target=0;
        }
        if(turret_target>360){
            turret_target=360;
        }
        if(goal_aiming){
            if(!use_only_heading_for_tracking){
                turretController.setTargetPosition(turret_target);
            }
        }
        else{
            turretController.setTargetPosition(TurretController.center_target_position);
        }
        switch (state){
            case INTAKING_1:
                if(intakeController.getMILLIAMPS_transfer()>3500 && intakeController.getTransferVelocity()>1550 && intakeController.getTransferVelocity()<1800){
                    intakeController.setState(IntakeController.State.IN_TAKING_SC_TH_BALL);
                    setState(State.INTAKING_2);
                }
                else{
                    intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
                    setState(State.INTAKING_1);
                }
                break;
            case INTAKING_2:
 
 
 
 
 
 
 
                break;
            case SHOOT_NEAR:
                if(shooterController.isShooterAtTarget()){
                    shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
                }
                if(timer.milliseconds()>700 && shooterController.isShooterAtTarget()){
                    intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
                    timer.reset();
                    setState(State.SHOOT_NEAR_2);
                }
                break;
            case SHOOT_NEAR_2:
                if(timer.milliseconds()>2000){
                    shooterController.setShooterVelocity(0);
                    shooting=false;
                    intaking();
                }
                break;
            case JUST_PUSH_BALLS_TO_TRANSFER:
                if(timer.milliseconds()>500){
                    intakeController.setState(IntakeController.State.ZERO);
                    setState(State.SHOOT_WHERE_EVER);
                }
                break;
            case SHOOT_WHERE_EVER:
                if(shooterController.isShooterAtTarget()){
                    shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
                    intakeController.setState(IntakeController.State.ZERO);
                }
                if(timer.milliseconds()>80 && shooterController.isShooterAtTarget()){
                    if(!far_zone){
                    intakeController.setState(IntakeController.State.FULL_POWER);}
                    if(far_zone){
                        intakeController.setState(IntakeController.State.SUPPLYING_FOR_FAR_ZONE);
                    }
                    timer.reset();
                    setState(State.SHOOT_WHERE_EVER_2);
                }
                break;
            case SHOOT_WHERE_EVER_2:
                if(!far_zone) {
                    if (timer.milliseconds() > 600 ) {
                        intaking();
                    }
                }
                if(far_zone){
                    if(timer.milliseconds()>1000){
                        intaking();
                    }
                }
                break;
            case JUST_PUSH_BALLS_TO_TRANSFER_AUTO:
                if(timer.milliseconds()>100){
                    intakeController.setState(IntakeController.State.ZERO);
                    setState(State.SHOOT_WHERE_EVER_AUTO);
                }
                break;
            case SHOOT_WHERE_EVER_AUTO:
                if(shooterController.isShooterAtTarget()){
                    shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
                    intakeController.setState(IntakeController.State.ZERO);
                }
                if(timer.milliseconds()>80 && shooterController.isShooterAtTarget()){
                    intakeController.setState(IntakeController.State.FULL_POWER);
                    timer.reset();
                    setState(State.SHOOT_WHERE_EVER_2_AUTO);
                }
                break;
            case SHOOT_WHERE_EVER_2_AUTO:
                if(timer.milliseconds()>500){
                    shooting=false;
                    setState(State.SHOOT_ENDED_AUTO);
                }
                break;
            case RESET_BLUE:
                if(timer.milliseconds()>250){
                    offset_for_tracking=-2;
                    follower.setPose(Pose_states.blue_Human_reset);
                    setState(State.ZERO);
                }
                break;
            case RESET_RED:
                if(timer.milliseconds()>250){
                    offset_for_tracking=-2;
                    follower.setPose(Pose_states.red_Human_reset);
                    setState(State.ZERO);
                }
                break;  
            case RESET_RED_GOAL:
                if(timer.milliseconds()>250){
                    offset_for_tracking=-2;
                    follower.setPose(Pose_states.red_goal_reset);
                    setState(State.ZERO);
                }
                break;
            case RESET_NEAR_BLUE_GOAL:
                if(timer.milliseconds()>250){
                    offset_for_tracking=-2;
                    follower.setPose(Pose_states.blue_goal_near_reset);
                    setState(State.ZERO);
                }
                break;
            case RESET_RED_GATE:
                if(timer.milliseconds()>250){
                    offset_for_tracking=-2;
                    follower.setPose(Pose_states.red_gate_reset);
                    setState(State.ZERO);
                }
                break;
            case RESET_BLUE_GATE:
                if(timer.milliseconds()>250){
                    offset_for_tracking=-2;
                    follower.setPose(Pose_states.blue_gate_reset);
                    setState(State.ZERO);
                }
                break;
            default:
                break;
        }
        shooterController.update();
        intakeController.update();
        turretController.update();
    }

    public void setState(State state){
        timer.reset();
        this.state=state;
    }
    public void debug(Telemetry telemetryA){
        telemetryA.addData("Shooter AMPS 1",shooterController.getShooter1_Amps());
        telemetryA.addData("Shooter AMPS 2",shooterController.getShooter2_Amps());
        telemetryA.addData("state",state);
        telemetryA.addData("FAR ZONE", far_zone);
        telemetryA.addData("Is it in ZONE",is_it_near_zones);
        telemetryA.addData("GOAL X",goal.getX());
        telemetryA.addData("GOAL Y",goal.getY());
        telemetryA.addData("x_velocity",x_velocity);
        telemetryA.addData("y_velocity",y_velocity);
        telemetryA.addData("turret_target",turret_target);
        telemetryA.addData("approx SHOOTER VELOCITY", shooter_speed_near_zone(hypotenus));
        telemetryA.addData("real SHOOTER VELOCITY",shooterController.currentVelocity);
        telemetryA.addData("is Shooter ON Target Position", shooterController.isShooterAtTarget());
        telemetryA.addData("approx HOOD POSITION", hood_degrees_near_zone(hypotenus));
        telemetryA.addData("REAL_POSITION_FROM_SERVO", shooterController.getHoodPosition());
        telemetryA.addData("approx HOOD COMPENSATION AMOUNT", hood_compensation);
        telemetryA.addData("hypotenus",hypotenus);
        telemetryA.addData("x inches to the red goal",x_inches_to_the_goal);
        telemetryA.addData("y inches to the red goal",y_inches_to_the_goal);
        telemetryA.addData("tan angle", arc_tan_to_the_red_goal);
        telemetryA.addData("current heading", currentHeading);
        telemetryA.addData("atan2",atan2);
    }

}
