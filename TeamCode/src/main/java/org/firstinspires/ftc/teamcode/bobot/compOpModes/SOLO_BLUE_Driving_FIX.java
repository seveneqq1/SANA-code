package org.firstinspires.ftc.teamcode.bobot.compOpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.controllers.HeadingController;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterControllerjana;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;
import org.firstinspires.ftc.teamcode.bobot.states.Pose_states;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp( group = "!a")
public class SOLO_BLUE_Driving_FIX extends OpMode {
    private GamepadEx first_driver,second_driver;
    private ElapsedTime runtime;
    private List<LynxModule> allHubs;
    private ElapsedTime timer;
    private VoltageSensor voltageSensor;
    public static int target_heading=0;
    HeadingController headingController;
    private Telemetry telemetryA;
    private MecanumDrive mecanumDrive;

    IntakeController intakeController;
    ShooterControllerjana shooterController;
    Follower follower;
    TurretController turretController;
    StateController stateController;
    Motor frontLeft ;
    Motor frontRight ;
    Motor backLeft ;
    Motor backRight ;
    private static double multiplier;

     
    private ElapsedTime brakeTimer = new ElapsedTime();
    private ElapsedTime autoBrakeTimer = new ElapsedTime();
    private boolean brakePulseFired = false;
    private boolean autoBrakeArmed = false;
    private boolean autoBrakeWatchingRelease = false;
    private boolean autoBrakeActive = false;
    private double lastStrafe = 0, lastForward = 0, lastTurn = 0;
    private double brakeStrafe = 0, brakeForward = 0, brakeTurn = 0;
    private static final double BRAKE_TRIGGER_THRESHOLD = 0.02;   
    private static final double BRAKE_PULSE_POWER_RATIO = 0.32;   
    private static final double BRAKE_PULSE_DURATION_SEC = 0.12;  
    private static final double DRIVE_INPUT_DEADBAND = 0.02;
    private static final double AUTO_BRAKE_ARM_INPUT = 0.45;
    private static final double AUTO_BRAKE_RELEASE_INPUT = 0.12;
    private static final double AUTO_BRAKE_FAIL_DELAY_SEC = 0.02;
    private static final double AUTO_BRAKE_WATCH_WINDOW_SEC = 0.25;
    private static final double AUTO_BRAKE_FAIL_VELOCITY = 6.0;
    private static final double AUTO_BRAKE_DONE_VELOCITY = 2.0;

    @Override
    public void init() {
        second_driver=new GamepadEx(gamepad2);
        first_driver =new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Pose_states.startPose_training);
        follower.updatePose();
        headingController=new HeadingController(hardwareMap);
        frontLeft = new Motor(hardwareMap, "leftFront");
        frontRight = new Motor(hardwareMap, "rightFront");
        backLeft = new Motor(hardwareMap, "leftBack");
        backRight = new Motor(hardwareMap, "rightBack");
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        mecanumDrive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);
        intakeController=new IntakeController(hardwareMap);
        shooterController=new ShooterControllerjana(hardwareMap);
        turretController=new TurretController(hardwareMap);
        stateController=new StateController(hardwareMap,shooterController,intakeController,turretController,follower, Pose_states.startPose_Blue_TeleOp,false);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        runtime=new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer=new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

    }
    @Override
    public void init_loop(){

    }

    @Override
    public void start() {
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        stateController.start();
        runtime.reset();
        timer.reset();
    }

    private double lastTimeStamp = 0;
    public static boolean make_it_heading=false;
    @Override
    public void loop() {
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        headingController.setTargetHeading(180+47-5);
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();

         
         
        follower.updatePose();

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0)
            lastTimeStamp = currentTimeStamp;
        double dt = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;
        telemetry.addData("voltage",voltageSensor.getVoltage());
        first_driver.readButtons();
        second_driver.readButtons();

        double turnSpeed = first_driver.getRightX();
        double strafePower = first_driver.getLeftX();
        double forwardPower = first_driver.getLeftY();
        if(first_driver.wasJustPressed(GamepadKeys.Button.X)){
            stateController.intaking();
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.A)){
            intakeController.setState(IntakeController.State.ZERO);
            shooterController.setShooterVelocity(0);
            shooterController.setSupplyState(ShooterControllerjana.BlockStates.BLOCK);
            stateController.setState(StateController.State.ZERO);
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.B)){
            stateController.balls_out();
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            stateController.move_turret(2 );
        } if(first_driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            stateController.move_turret(-2);
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.Y)){
            stateController.shoot_whereever_you_want();
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            stateController.reset_blue_human();
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            stateController.turret_state_change();
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) ){
            stateController.reset_blue_gate();
        }
        if(first_driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            stateController.reset_near_blue_goal();
        }
        stateController.update();
        if(second_driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            stateController.switch_to_manual_mode();
        }
        if(second_driver.wasJustPressed(GamepadKeys.Button.Y)){
            stateController.shoot_manual_middle();
        }
        if(second_driver.wasJustPressed(GamepadKeys.Button.X)){
            stateController.shoot_manual_near();
        }
        if(second_driver.wasJustPressed(GamepadKeys.Button.B)){
            stateController.shoot_manual_far_middle();
        }
        if(second_driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            stateController.move_turret(4);
        } if(second_driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            stateController.move_turret(-4);
        }
        if (first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
            multiplier=0.6;
        }
        else{
            multiplier=1;
        }
        if(stateController.switch_to_manual_mode){
            second_driver.gamepad.rumble(99999999);
        }
        stateController.debug(telemetry);

        boolean manualBrake = first_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > BRAKE_TRIGGER_THRESHOLD
                || gamepad1.left_trigger > BRAKE_TRIGGER_THRESHOLD;
        boolean emergencyBrake = manualBrake;
        double currentStrafe = strafePower * multiplier;
        double currentForward = forwardPower * multiplier;
        double currentTurn = turnSpeed * multiplier;
        double driveInputMagnitude = Math.hypot(currentStrafe, currentForward);
        double turnInputMagnitude = Math.abs(currentTurn);
        double totalInputMagnitude = Math.max(driveInputMagnitude, turnInputMagnitude);

         
         
        double robotVelocity = follower.getVelocity().getMagnitude();

        if (totalInputMagnitude > AUTO_BRAKE_ARM_INPUT) {
            autoBrakeArmed = true;
            autoBrakeWatchingRelease = false;
            autoBrakeActive = false;
        } else if (totalInputMagnitude > AUTO_BRAKE_RELEASE_INPUT) {
            autoBrakeWatchingRelease = false;
            autoBrakeActive = false;
        } else if (autoBrakeArmed && !autoBrakeWatchingRelease && !autoBrakeActive) {
            autoBrakeWatchingRelease = true;
            autoBrakeTimer.reset();
        }

        if (autoBrakeWatchingRelease
                && autoBrakeTimer.seconds() > AUTO_BRAKE_FAIL_DELAY_SEC
                && robotVelocity > AUTO_BRAKE_FAIL_VELOCITY) {
            autoBrakeActive = true;
            autoBrakeWatchingRelease = false;
        }

        if (autoBrakeWatchingRelease && autoBrakeTimer.seconds() > AUTO_BRAKE_WATCH_WINDOW_SEC) {
            autoBrakeWatchingRelease = false;
            autoBrakeArmed = false;
        }

        if ((autoBrakeActive && robotVelocity < AUTO_BRAKE_DONE_VELOCITY) || manualBrake) {
            autoBrakeActive = false;
            autoBrakeArmed = false;
        }

        emergencyBrake = emergencyBrake || autoBrakeActive;

        if (emergencyBrake) {
            if (!brakePulseFired) {
                brakePulseFired = true;
                brakeTimer.reset();
                brakeStrafe = Math.abs(currentStrafe) > DRIVE_INPUT_DEADBAND ? currentStrafe : lastStrafe;
                brakeForward = Math.abs(currentForward) > DRIVE_INPUT_DEADBAND ? currentForward : lastForward;
                brakeTurn = Math.abs(currentTurn) > DRIVE_INPUT_DEADBAND ? currentTurn : lastTurn;
            }
            if (brakeTimer.seconds() < BRAKE_PULSE_DURATION_SEC) {
                 
                mecanumDrive.driveFieldCentric(
                        -brakeStrafe * BRAKE_PULSE_POWER_RATIO,
                        -brakeForward * BRAKE_PULSE_POWER_RATIO,
                        -brakeTurn * BRAKE_PULSE_POWER_RATIO,
                        0);
            } else {
                 
                mecanumDrive.driveFieldCentric(0, 0, 0, 0);
            }
        } else {
            brakePulseFired = false;
            if (Math.abs(currentStrafe) > DRIVE_INPUT_DEADBAND) {
                lastStrafe = currentStrafe;
            }
            if (Math.abs(currentForward) > DRIVE_INPUT_DEADBAND) {
                lastForward = currentForward;
            }
            if (Math.abs(currentTurn) > DRIVE_INPUT_DEADBAND) {
                lastTurn = currentTurn;
            }

            if(make_it_heading) {
                mecanumDrive.driveFieldCentric(currentStrafe,currentForward,-headingController.calculate(),0);
            }
            if(!make_it_heading) {
                mecanumDrive.driveFieldCentric(currentStrafe,currentForward,currentTurn,0);
            }
        }

 
 
 
 
        telemetry.update();
    }
}