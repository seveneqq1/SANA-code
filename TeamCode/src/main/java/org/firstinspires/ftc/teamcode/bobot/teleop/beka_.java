package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.bobot.controller.SnapStateMachine;
import org.firstinspires.ftc.teamcode.bobot.controller.intakeController;
import org.firstinspires.ftc.teamcode.bobot.controller.shooterController_var2;
import org.firstinspires.ftc.teamcode.bobot.controller.turretController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

import dev.frozenmilk.util.collections.Cons;

public class beka_ extends OpMode {
    private GamepadEx first_driver, second_driver;
    private List<LynxModule> allHubs;
    private VoltageSensor voltageSensor;
    intakeController IntakeController;
    shooterController_var2 ShooterController;
    turretController TurretController;
    SnapStateMachine snapStateMachine;
    Follower follower;
    Motor frontLeft, frontRight, backLeft, backRight;
    Pose startPose;
    private static double multiplier;
    @Override
    public void init(){
        first_driver = new GamepadEx(gamepad1);
        second_driver = new GamepadEx(gamepad2);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        frontLeft = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft = new Motor(hardwareMap, "backleft");
        backRight = new Motor(hardwareMap, "backright");
        frontRight.setInverted(true);
        backRight.setInverted(true);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.set(0);
        frontLeft.set(0);
        backRight.set(0);
        backLeft.set(0);
        shooterController_var2 ShooterController = new shooterController_var2(hardwareMap);
        intakeController IntakeController = new intakeController(hardwareMap);
        turretController TurretController = new turretController(hardwareMap);
        snapStateMachine = new SnapStateMachine(ShooterController, IntakeController, TurretController, null);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){
//        snapStateMachine.start();
    }
    @Override
    public void loop(){
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
        first_driver.readButtons();
        second_driver.readButtons();
        follower.update();
        if(first_driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            follower.setPose(startPose);
        }
    }
}
