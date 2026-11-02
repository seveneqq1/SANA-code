package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@TeleOp(name = "calibration")
public class PinpointOffsetCalibration extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode(){
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,0));
        telemetry.addData("status","initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            follower.update();
            if(gamepad1.a){
                follower.manual(0, 0, -0.3);
            } else if (gamepad1.b){
                follower.manual(0,0,0.3);
            } else {
                follower.manual(0,0,0);
            }
            Pose currentPose = follower.pose();
            telemetry.addData("X pos", currentPose.x());
            telemetry.addData("Y pos", currentPose.y());
            telemetry.addData("heading", currentPose.heading());
            telemetry.update();
        }
    }
}
