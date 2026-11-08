package org.firstinspires.ftc.teamcode.bobot.teleop;

/**
 * Early Blue TeleOp sketch. Prefer RegionalATeleOp for matches.
 * Drive is robot-centric on gamepad1; shooter/intake are constructed but not sequenced.
 */

import org.firstinspires.ftc.teamcode.bobot.controller.intakeController;
import org.firstinspires.ftc.teamcode.bobot.controller.shooterController;
import org.firstinspires.ftc.teamcode.bobot.controller.turretController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import java.util.List;

@TeleOp(name = "Blue")
public class blue extends OpMode {
    private GamepadEx first_driver, second_driver;
    private List<LynxModule> allHubs;
    intakeController IntakeController;
    shooterController ShooterController;
    private MecanumDrive mecanumDrive;
    Follower follower;
    Pose startPose = new Pose(0, 0, Math.PI); // heading in radians
    turretController TurretController;

    Motor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init(){
        IntakeController = new intakeController(hardwareMap);
        ShooterController = new shooterController(hardwareMap);
        TurretController = new turretController(hardwareMap);
        first_driver = new GamepadEx(gamepad1);
        second_driver = new GamepadEx(gamepad2);
        follower = Constants.create(hardwareMap);
        if(follower != null) {
            follower.setPose(startPose);
            follower.update();
        }
        frontLeft = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft = new Motor(hardwareMap, "backleft");
        backRight = new Motor(hardwareMap, "backright");
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mecanumDrive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){
//        stateController.start();
    }

    @Override
    public void loop(){
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
        first_driver.readButtons();
        second_driver.readButtons();
        double rx = first_driver.getRightX();
        double x = first_driver.getLeftX();
        double y = first_driver.getLeftY();
        mecanumDrive.driveRobotCentric(x, y, rx);
        if (follower != null) {
            follower.update();
        }
    }
}
