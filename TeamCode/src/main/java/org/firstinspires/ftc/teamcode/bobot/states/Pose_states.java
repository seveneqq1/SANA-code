package org.firstinspires.ftc.teamcode.bobot.states;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Pose_states {

    public static boolean isRedAlliance = true;

    public final Pose startPose_AUTO_RED_NEAR = new Pose(112.4, 134.5, Math.toRadians(90));
    public final Pose startPose_AUTO_RED_FAR=new Pose(92.5,7.5,0);
    public final Pose startPose_AUTO_BLUE_NEAR = new Pose(31.4, 134.5 , Math.toRadians(90));
    public final Pose startPose_AUTO_BLUE_FAR=new Pose(51.5,7.5,Math.toRadians(180));
    public static final Pose startPose_training=new Pose(79.1,9,Math.toRadians(90));
    public static final Pose startPose_Red_TeleOp=new Pose(126,90,0);
    public static final Pose open_gate_red=new Pose(124.6,61,Math.toRadians(25));  
    public static final Pose open_gate_blue=new Pose(13.67,63.5,Math.toRadians(151.26));
    public static final Pose score_pose_for_blue=new Pose(58,80,Math.toRadians(197));
    public static final Pose startPose_Blue_TeleOp=new Pose(18,90,Math.toRadians(180));
    public static final Pose red_Human_reset=new Pose(11,13,Math.toRadians(180));
    public static final Pose red_goal_reset=new Pose(125,110.3,Math.toRadians(Math.toRadians(353)));
    public static final Pose blue_goal_near_reset=new Pose(18.1807,109.8,Math.toRadians(187));
    public static final Pose red_gate_reset=new Pose(125.7,64,0);
    public static final Pose blue_gate_reset=new Pose(18.7,65.7,Math.toRadians(180));
    public static final Pose blue_Human_reset=new Pose(133,7.64,0);
    public static Pose currentPose=new Pose(0,0,0);
    private final double robot_length=17.07;
    private final double robot_width=14.33;
 
 
public static final Pose red_goal_near =new Pose(138, 138,0);
    public static final Pose blue_goal_near =new Pose(6,138,0);
    public static final Pose blue_goal_far =new Pose(0,144,0);
    public static final Pose red_goal_far =new Pose(140,144,0);
     
      
      

}
