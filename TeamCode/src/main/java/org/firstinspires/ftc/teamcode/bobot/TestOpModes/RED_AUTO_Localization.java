package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.bobot.utlities.MyMath;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class RED_AUTO_Localization extends OpMode {
    private Follower follower;
    public static Pose startingPose=new Pose(71,9,Math.toRadians(90));  
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    TurretController turretController;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        turretController=new TurretController(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125,110,0));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()  
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(84, 84))))
                .setLinearHeadingInterpolation(follower.getHeading(),Math.toRadians(0))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    public static double redd_offset_for_tracking=0;
    double turret_target;
    @Override
    public void loop() {
         
        follower.update();
        telemetryM.update();
        double x_inches_to_the_red_goal=138-follower.getPose().getX();
        double y_inches_to_the_red_goal=138-follower.getPose().getY();
        double arc_tan_to_the_red_goal=MyMath.arcTan(y_inches_to_the_red_goal/x_inches_to_the_red_goal);
        double currentRaw=follower.getPose().getHeading()/6.28 * 360.0;
        if(currentRaw<0){
            currentRaw=360+currentRaw;
        }
        if (currentRaw > 0 && currentRaw < 40) {
            turret_target = Math.abs(currentRaw - arc_tan_to_the_red_goal) +redd_offset_for_tracking;
        } else {
            turret_target = 360 - (currentRaw - arc_tan_to_the_red_goal) +redd_offset_for_tracking;
        }
        turretController.setTargetPosition(turret_target);
        turretController.update();
        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true  
            );
        }

         
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

         
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

         
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

         
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

         
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }
        telemetryM.addData("turret_ target", turret_target);
        telemetryM.addData("Heading",currentRaw);
        telemetryM.addData("x",follower.getPose().getX());
        telemetryM.addData("y",follower.getPose().getY());
        telemetryM.update();
    }
}
