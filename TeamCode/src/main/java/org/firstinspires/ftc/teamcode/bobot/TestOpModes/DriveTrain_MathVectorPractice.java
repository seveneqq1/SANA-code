package org.firstinspires.ftc.teamcode.bobot.TestOpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterControllerjana;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.bobot.utlities.MyMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;




@Configurable
@TeleOp(group = "Teleop Test")
public class DriveTrain_MathVectorPractice extends OpMode {
    public static double shooter_target=0;
    public static double hood_position=35;
    public static double turret_target;

    Follower follower;
    IntakeController intakeController;
    TurretController turretController;
    ShooterControllerjana shooterController;
    private Telemetry telemetryA;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;
     


    private enum  State{
        ZERO,SCORE_BALLS,SCORE_BALLS_2,SCORE_BALLS_3,
        INTAKING;
    }
    private State state=State.ZERO;
    private ElapsedTime timer;
    @Override
    public void init() {
        timer=new ElapsedTime();
        intakeController=new IntakeController(hardwareMap);
        turretController=new TurretController(hardwareMap);
        shooterController=new ShooterControllerjana(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Pose_states.startPose_training);
        follower.update();

 
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        follower.setPose(Pose_states.startPose_training);

    }
    @Override
    public void start(){

    }
    double hypotenus=0;
    double y_inches_to_the_goal,x_inches_to_the_goal;
    double real_x,real_y;
    Pose real_goal=Pose_states.red_goal_near;
    public static double hood_compstn(double dist){
        return MathFunctions.clamp(-5*dist+1065,0,2000);
    }
    public static double hood_compensation_multiplier =17;
    public static double turret_offset=0;
    @Override
    public void loop() {
        double compensation=(shooterController.getTargetPositon()-shooterController.getCurrentVelocity());
        double hood_compensation=(compensation)/ hood_compensation_multiplier;
        double x_inches_to_the_red_goal=138-follower.getPose().getX();
        double y_inches_to_the_red_goal=138-follower.getPose().getY();
        double arc_tan_to_the_red_goal=MyMath.arcTan(y_inches_to_the_red_goal/x_inches_to_the_red_goal);
        double currentRaw=follower.getPose().getHeading()/6.28 * 360.0;
        if(currentRaw<0){
            currentRaw=360+currentRaw;
        }
        if (currentRaw > 0 && currentRaw < 40) {
            turret_target = Math.abs(currentRaw - arc_tan_to_the_red_goal);
        } else {
            turret_target = 360 - (currentRaw - arc_tan_to_the_red_goal);
        }
        turretController.setTargetPosition(turret_target+turret_offset);
        shooterController.setShooterVelocity(shooter_target);
        shooterController.setRampPosition(hood_position-hood_compensation);
        shooterController.update();
        turretController.update();
        intakeController.update();

        x_inches_to_the_goal= real_goal.getX()-follower.getPose().getX();
        y_inches_to_the_goal= real_goal.getY()-follower.getPose().getY();
        boolean far_zone= follower.getPose().getY() < 56;
        if(!far_zone){
        real_goal=Pose_states.red_goal_near;
        }
        if(far_zone){
            real_goal=Pose_states.red_goal_far;
        }
        real_x=follower.getPose().getX();
        real_y=follower.getPose().getY();
        hypotenus= MyMath.pifagor(x_inches_to_the_goal,y_inches_to_the_goal);
        double y = -gamepad1.left_stick_y;  
        double x = gamepad1.left_stick_x;  
        double rx = gamepad1.right_stick_x;

         
         
         
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
        telemetry.addData("shooter AMPS", shooterController.getShooter1_Amps()+shooterController.getShooter2_Amps());
        telemetry.addData("power shooter",shooterController.power);
        telemetry.addData("target position",shooterController.getTargetPositon());
        telemetry.addData("currentPosition",shooterController.getCurrentVelocity());
        telemetry.addData("hood_compensation in degrees",hood_compensation);
        telemetry.addData("hypotenus",hypotenus);
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("Y",follower.getPose().getY());
        telemetry.addData("x inches to the red goal",x_inches_to_the_goal);
        telemetry.addData("y inches to the red goal",y_inches_to_the_goal);
        telemetry.addData("tan angle", arc_tan_to_the_red_goal);
        telemetry.update();
        follower.updatePose();
        if(gamepad1.x){
            intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
           shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
        }
        if(gamepad1.y){
            intakeController.setState(IntakeController.State.ZERO);
            shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
            timer.reset();
            state=State.SCORE_BALLS;
        }
        switch (state){
            case SCORE_BALLS:
                if(timer.milliseconds()>500){
                    intakeController.setState(IntakeController.State.FULL_POWER);
                    timer.reset();
                    state=State.SCORE_BALLS_2;
                }
                break;
            case SCORE_BALLS_2:
                if(timer.milliseconds()>800){
                    intakeController.setState(IntakeController.State.ZERO);
                    shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
                    timer.reset();
                    state=State.SCORE_BALLS_3;
                }
                break;
            case SCORE_BALLS_3:
                if(timer.milliseconds()>100){
                    intakeController.setState(IntakeController.State.IN_TAKING_FULL_POWER);
                    state=State.ZERO;
                }
                break;
            default:
                break;
        }
        if(gamepad1.dpad_down){
            intakeController.setState(IntakeController.State.SUPPLYING_FOR_FAR_ZONE);
            shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
        }
        if(gamepad1.b){
            intakeController.setState(IntakeController.State.BALLS_OUT);
        }
        if(gamepad1.dpad_up){
            shooterController.setSupplyState(ShooterControllerjana.BlockStates.SUPPLY);
        }
        if(gamepad1.a) {
            intakeController.setState(IntakeController.State.ZERO);
            shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
        }
    }
}
