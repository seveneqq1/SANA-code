package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.states.ModeStates;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.bobot.utlities.MyMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public class StateControllerAuto {

    public ElapsedTime timer;
    public ElapsedTime deviate_timer;
    Follower follower;
    ShooterControllerjana shooterController;
    IntakeController intakeController;
    TurretController turretController;
    public boolean isShooterAlwayson=true;
    public boolean use_only_heading_for_tracking=false;
    public static double blue_offset_for_tracking_with_only_heading=20;
    public static double offset_for_tracking=2;
    public static double blue_offset_for_tracking=14;
    public static double tangent_offset=35;
    private State state = State.ZERO;
    public enum State{
        ZERO,
        INTAKING_1,INTAKING_2,
         JUST_PUSH_BALLS_TO_TRANSFER_AUTO,
        SHOOT_WHERE_EVER_AUTO,SHOOT_WHERE_EVER_2_AUTO,SHOOT_ENDED_AUTO,

    }
    public State getState() {
        return state;
    }
    public boolean auto;
    private boolean redd;
    public StateControllerAuto(HardwareMap hardwareMap, ShooterControllerjana shooterController, IntakeController intakeController, TurretController turretController, Follower follower, boolean red) {
        this.shooterController=shooterController;
        this.intakeController=intakeController;
        this.turretController=turretController;
        this.follower=follower;
        timer=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
    public void start(){
        turretController.setTargetPosition(TurretController.center_target_position);
        turretController.update();
    }
    public boolean goal_aiming=true;
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

    public void intaking(){
        intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
        shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
        setState(State.INTAKING_1);
    }
    public void shoot_whereever_you_want_auto(){
        if(state== State.ZERO|| state==State.INTAKING_1 || state==State.INTAKING_2){
            timer.reset();
            shooting=true;
            timer.reset();
            intakeController.setState(IntakeController.State.ZERO);
            setState(State.JUST_PUSH_BALLS_TO_TRANSFER_AUTO);
        }
    }
    public boolean is_it_shoot_while_move=true;

    public double getHeading() {
        return currentHeading;
    }
    public static boolean use_hypotenus=true;
    public static double offset_velocity=0;
    public static double hood_comp_multiply=1.75;
    public void update(){
        if(shooting || isShooterAlwayson){
            if(!far_zone) {
                double compensation = (shooterController.getTargetPositon() - shooterController.getCurrentVelocity());
                hood_compensation = ((compensation) / hood_comp(hypotenus)*hood_comp_multiply);
                shooterController.setShooterVelocity(shooter_speed_near_zone(hypotenus)+10+offset_velocity);
 
                shooterController.setRampPosition(hood_degrees_near_zone(hypotenus) - hood_compensation);  
                if (hood_compensation < 0) {
                    hood_compensation = 0;
                }
            }
        }
        if(far_zone){
            double compensation=(shooterController.getTargetPositon()-shooterController.currentVelocity);
            hood_compensation=(compensation)/hood_comp_far_zone(hypotenus);
            shooterController.setShooterVelocity(shooter_speed_far_zone(hypotenus));
            if(!shooting) {
                shooterController.setRampPosition(hood_degrees_far_zone(hypotenus));






            }
            if(shooting && hood_compensation>=0){
                shooterController.setRampPosition(hood_degrees_far_zone(hypotenus)-hood_compensation);
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
        if(use_hypotenus){
        hypotenus= MyMath.pifagor(x_inches_to_the_goal,y_inches_to_the_goal);}

        if(redd){
            if(!use_only_heading_for_tracking) {
                if (currentRaw > 0 && currentRaw < 35) {
                    turret_target = Math.abs(currentRaw - arc_tan_to_the_red_goal) +offset_for_tracking;
                } else {
                    turret_target = 360 - (currentRaw - arc_tan_to_the_red_goal) +offset_for_tracking;
                }
            }
            if(use_only_heading_for_tracking){
                if (currentRaw > 0 && currentRaw < 35) {
                    turret_target = Math.abs(currentRaw - tangent_offset ) - 1;
                } else {
                    turret_target = 360 - (currentRaw - tangent_offset) - 1;
                }
            }
        }
        if(!redd){
            if(!use_only_heading_for_tracking) {
 
 
 
 
 
                turret_target = 180-currentRaw - arc_tan_to_the_red_goal+offset_for_tracking+2;
                turret_target=turret_target%360;
                if(turret_target<0){
                    turret_target+=360;
                }
            }
            if(use_only_heading_for_tracking){
                if (currentRaw > 0 && currentRaw < 145) {
                    turret_target = 180 - currentRaw - arc_tan_to_the_red_goal + blue_offset_for_tracking;
                } else {
                    turret_target = 360 - (currentRaw - arc_tan_to_the_red_goal - 90) +blue_offset_for_tracking;
                }
                if (far_zone) {
                    turret_target += 4;
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
            if(!far_zone){
                shooterController.setRampPosition(hood_degrees_near_zone(hypotenus));
            }
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
            case JUST_PUSH_BALLS_TO_TRANSFER_AUTO:
                if(timer.milliseconds()>10){
                    intakeController.setState(IntakeController.State.ZERO);
                    setState(State.SHOOT_WHERE_EVER_AUTO);
                }
                break;
            case SHOOT_WHERE_EVER_AUTO:
                if(shooterController.isShooterAlmsotAtTarget()){
                    shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
                    intakeController.setState(IntakeController.State.ZERO);
                }
                if(timer.milliseconds()>80 && shooterController.isShooterAlmsotAtTarget()){
                    if(!far_zone){
                    intakeController.setState(IntakeController.State.FULL_POWER);}
                    if(far_zone){
                        intakeController.setState(IntakeController.State.SUPPLYING_FOR_FAR_ZONE);
                    }
                    timer.reset();
                    setState(State.SHOOT_WHERE_EVER_2_AUTO);
                }
                break;
            case SHOOT_WHERE_EVER_2_AUTO:
                if(timer.milliseconds()>350 && !far_zone){
                    shooting=false;
                    setState(State.SHOOT_ENDED_AUTO);
                }
                if(timer.milliseconds()>800 && far_zone){
                    shooting=false;
                    setState(State.SHOOT_ENDED_AUTO);
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
