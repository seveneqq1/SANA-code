package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController1_without_limelight_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.TransferController_new;
import org.firstinspires.ftc.teamcode.bobot.states.poseStates;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(name = "Blue TeleOp without limelight new", group = "!")
public class blue_without_limelight_new extends LinearOpMode {

    private static final double SLOW_MODE_SCALE = 0.4;
    private static final double DRIVE_CURVE_EXPONENT = 2.0;  
    private static final double TRIGGER_THRESHOLD = 0.6;
    private static final double SHOOTER_READY_TOLERANCE = 0.05;  
    private static final int TELEMETRY_INTERVAL_MS = 100;

     
    private static final double NUDGE_POWER = 0.8;
    private static final long NUDGE_DURATION_MS = 40;

     
    private static final double LEFT_GOAL_X = 60.0;
    private static final double LEFT_GOAL_Y = 84.0;
    private static final double RIGHT_GOAL_X = 84.0;
    private static final double RIGHT_GOAL_Y = 84.0;
    private static final double RESET_CORNER_X = 137.0;
    private static final double RESET_CORNER_Y = 137.0;
    private static final double HEADING_P_GAIN = 2.2;

    private GamepadEx first_driver, second_driver;
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    private ShooterController_new shooterController;
    private IntakeController_new intakeController;
    private TransferController_new transferController;
    private StateController1_without_limelight_new stateController;
    private Follower follower;

    private boolean slowMode = false;
    private boolean shooterWasReady = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private double turnNudgePower = 0.0;
    private long turnNudgeEndTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
         
         
        initSubsystems();
        initHardware();

        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

         
        stateController.spinUpBoth();

        while (opModeIsActive()) {
            first_driver.readButtons();
            second_driver.readButtons();
             
             
            stateController.update();

            stateController.spinUpBoth();  

            handleLocalizationReset();
            handleTriggerMicroTurn();
            drive();
            handleIntake();
            handleShooting();
            handleModeToggles();
            handleShooterReadyFeedback();
            handleFl();
            handleTransferseconddr();

            updateTelemetry();
        }

        stateController.stop_everything();
    }



    private void initSubsystems() {
        intakeController = new IntakeController_new(hardwareMap);
        transferController = new TransferController_new(hardwareMap);
        shooterController = new ShooterController_new(hardwareMap);

        first_driver = new GamepadEx(gamepad1);
        second_driver = new GamepadEx(gamepad2);

        try {
            follower = Constants.create(hardwareMap);
            if (follower != null) {
                follower.setPose(new poseStates().blue_starting_pose);
                follower.update();
            }
        } catch (Exception e) {
            follower = null;
        }

        stateController = new StateController1_without_limelight_new(
                follower, shooterController, intakeController, transferController, false, telemetry
        );
         
        stateController.enableAutoSideDetection();
    }
    private void initHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    private void handleLocalizationReset() {
        if (first_driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) ||
                second_driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (follower != null) {
                follower.setPose(new Pose(RESET_CORNER_X, RESET_CORNER_Y, Math.toRadians(180)));
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
        }
    }

    private void handleTriggerMicroTurn() {
        boolean rBumper = second_driver.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean lBumper = second_driver.getButton(GamepadKeys.Button.LEFT_BUMPER);

 
 
 
 
 
 
 

        lastRightBumper = rBumper;
        lastLeftBumper = lBumper;
    }

    private boolean isTrackingGoal() {
        return second_driver.getButton(GamepadKeys.Button.RIGHT_BUMPER);
    }

    private double calculateAutoTurnToGoal() {
        if (follower == null) return 0.0;

        Pose currentPose = follower.pose();
        if (currentPose == null) return 0.0;

         
        boolean left = stateController.isLeftSide();
        double targetX = left ? LEFT_GOAL_X : RIGHT_GOAL_X;
        double targetY = left ? LEFT_GOAL_Y : RIGHT_GOAL_Y;

        double targetHeadingRad = Math.atan2(targetY - currentPose.y(), targetX - currentPose.x()) + Math.PI/4;
        double currentHeadingRad = currentPose.heading();

        double headingError = Math.atan2(Math.sin(targetHeadingRad - currentHeadingRad),
                Math.cos(targetHeadingRad - currentHeadingRad));
        double turnPower = -headingError * HEADING_P_GAIN;
        return Math.max(-1.0, Math.min(1.0, turnPower));
    }

     
    private void drive() {
        double forward = curve(-gamepad1.left_stick_y);
        double strafe = curve(gamepad1.left_stick_x);
        double turn = curve(gamepad1.right_stick_x);

        if (isTrackingGoal()) {
            turn = calculateAutoTurnToGoal();
        } else if (System.currentTimeMillis() < turnNudgeEndTime) {
            turn += turnNudgePower;
        }

        double scale = slowMode ? SLOW_MODE_SCALE : 1.0;

        double lf = (forward + strafe + turn) * scale;
        double lb = (forward - strafe + turn) * scale;
        double rf = (forward - strafe - turn) * scale;
        double rb = (forward + strafe - turn) * scale;

        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(lb),
                Math.max(Math.abs(rf), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    private double curve(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), DRIVE_CURVE_EXPONENT);
    }

    private void handleIntake() {
        if (first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD || second_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD) {
            stateController.intake();
        } else if (first_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD || second_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD) {
            stateController.balls_out();
        } else if (stateController.getState() == StateController1_without_limelight_new.State.INTAKING
                || stateController.getState() == StateController1_without_limelight_new.State.PUSH_BALLS_OUT) {
            stateController.stop_intake();
        }
    }

    private void handleShooting() {
         
         
        if (second_driver.wasJustPressed(GamepadKeys.Button.Y)) {
            stateController.shootPollen();
        } else if (second_driver.wasJustPressed(GamepadKeys.Button.X)) {
            stateController.shootNectar();
        } else if (second_driver.wasJustPressed(GamepadKeys.Button.B)) {
            stateController.stop_intake();
        }

         
        if (first_driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            stateController.prepareShooter1();
        } else if (first_driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            stateController.prepareShooter2();
        }
    }
    private void handleModeToggles() {
        if (first_driver.wasJustPressed(GamepadKeys.Button.BACK)) {
            slowMode = !slowMode;
            gamepad1.rumble(100);
        }
    }

    private void handleFl() {
        if (second_driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            intakeController.setFlPos(0.9);
        } else if (second_driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            intakeController.setFlPos(0.2);
        }
    }

    private void handleTransferseconddr(){
        if(second_driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            transferController.setTransferPower(-0.75);
        }
    }

    private void handleShooterReadyFeedback() {
        boolean ready1 = withinTolerance(shooterController.getShooter1Velocity(), shooterController.getTargetVelocityRPM1());
        boolean ready2 = withinTolerance(shooterController.getShooter2Velocity(), shooterController.getTargetVelocityRPM2());
        boolean ready = ready1 || ready2;

        if (ready && !shooterWasReady) {
            gamepad1.rumble(150);
        }
        shooterWasReady = ready;
    }

    private boolean withinTolerance(double current, double target) {
        return target > 0 && Math.abs(current - target) < target * SHOOTER_READY_TOLERANCE;
    }

    private void updateTelemetry() {
        boolean left = stateController.isLeftSide();

        telemetry.addData("State", stateController.getState());
        telemetry.addData("Drive Mode", slowMode ? "ROBOT / SLOW" : "ROBOT");
        telemetry.addData("Side (auto)", left ? "LEFT" : "RIGHT");

        if (follower != null && follower.pose() != null) {
            Pose p = follower.pose();
            String targetStr = left
                    ? String.format("LEFT (%.0f, %.0f)", LEFT_GOAL_X, LEFT_GOAL_Y)
                    : String.format("RIGHT (%.0f, %.0f)", RIGHT_GOAL_X, RIGHT_GOAL_Y);

            telemetry.addData("Pose (X, Y, Deg)", "%.1f, %.1f, %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("Goal Tracking", isTrackingGoal() ? "ACTIVE -> " + targetStr : "OFF");
        }

        telemetry.addData("Shooter1 RPM", "%.0f / %.0f",
                shooterController.getShooter1Velocity(), shooterController.getTargetVelocityRPM1());
        telemetry.addData("Shooter2 RPM", "%.0f / %.0f",
                shooterController.getShooter2Velocity(), shooterController.getTargetVelocityRPM2());
        telemetry.addData("Stopper1 Config", "B:%.2f U:%.2f",
                ShooterController_new.STOPPER1_BLOCK_POS, ShooterController_new.STOPPER1_UNBLOCK_POS);
        telemetry.update();
    }
}