package org.firstinspires.ftc.teamcode.bobot.compOpModes;

import org.firstinspires.ftc.teamcode.bobot.config.ControlBindings;

import static org.firstinspires.ftc.teamcode.bobot.config.ControlBindings.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.controllers.DriveController;
import org.firstinspires.ftc.teamcode.bobot.controllers.FlowerTakerController;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;
import org.firstinspires.ftc.teamcode.bobot.utilities.ControlMath;
import org.firstinspires.ftc.teamcode.bobot.states.PoseStorage;

public abstract class RegionalTeleOp extends OpMode {
    private DriveController drive;
    private StateController stateController;
    private FlowerTakerController flowerTaker;
    private boolean usingAutoPose, intakeOn;
    private boolean wasTurretLeft, wasTurretRight, wasResetPose, wasFlowerTaker, wasIntake, wasShoot;

    protected abstract Alliance alliance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        stateController = new StateController(hardwareMap, alliance());
        drive = new DriveController(hardwareMap, stateController.voltage);
        flowerTaker = new FlowerTakerController(hardwareMap);

        Pose autoPose = null;
        if (RobotConfig.TELEOP_USE_AUTO_POSE) autoPose = PoseStorage.consume(alliance());
        else PoseStorage.clear();
        usingAutoPose = autoPose != null;
        drive.setPose(usingAutoPose ? autoPose : PoseStorage.startPose(alliance()));
        reportInit();
    }

    @Override
    public void init_loop() {
        drive.updateOdometry();
        drive.stop();
        stateController.stop();
        reportInit();
    }

    private void reportInit() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Pose source", usingAutoPose ? "autonomous handoff" : "RobotConfig start pose");
        telemetry.addData("Start pose", drive.getPose());
        telemetry.addData("Pose reset", drive.getPoseReset());
        telemetry.addLine("Verify start pose before START. DPAD RIGHT resets pose in your corner.");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.updateOdometry();
        boolean stop = held(STOP_DRIVER, STOP_BUTTON);
        boolean slow = held(SLOW_DRIVER, SLOW_BUTTON);
        boolean shoot = held(SHOOT_DRIVER, SHOOT_BUTTON);
        boolean eject = held(EJECT_DRIVER, EJECT_BUTTON);
        boolean intake = held(INTAKE_DRIVER, INTAKE_BUTTON);
        boolean resetPose = held(RESET_POSE_DRIVER, RESET_POSE_BUTTON);
        boolean turretLeft = held(TURRET_LEFT_DRIVER, TURRET_LEFT_BUTTON);
        boolean turretRight = held(TURRET_RIGHT_DRIVER, TURRET_RIGHT_BUTTON);
        boolean flower = held(FLOWER_TAKER_DRIVER, FLOWER_TAKER_BUTTON);

        if (resetPose && !wasResetPose) drive.setPose(PoseStorage.cornerPose(alliance()));
        if (turretLeft && !wasTurretLeft) stateController.turret.nudge(RobotConfig.TURRET_NUDGE_DEG);
        if (turretRight && !wasTurretRight) stateController.turret.nudge(-RobotConfig.TURRET_NUDGE_DEG);
        if (flower && !wasFlowerTaker) flowerTaker.toggle();
        if (intake && !wasIntake) intakeOn = !intakeOn;
        if (shoot && !wasShoot) intakeOn = true;
        wasFlowerTaker = flower;
        wasIntake = intake;
        wasShoot = shoot;
        wasResetPose = resetPose;
        wasTurretLeft = turretLeft;
        wasTurretRight = turretRight;

        Gamepad driver = ControlBindings.driver(DRIVE_DRIVER, gamepad1, gamepad2);
        if (stop) {
            intakeOn = false;
            drive.stop();
            stateController.reset();
            flowerTaker.close();
        } else {
            drive.drive(stick(-driver.left_stick_y), stick(-driver.left_stick_x), stick(-driver.right_stick_x), slow);

            if (eject) stateController.eject();
            else if (shoot) stateController.shoot();
            else if (intakeOn) stateController.intake();
            else stateController.idle();
        }
        stateController.setTurretAlwaysOn(!stop);
        stateController.update(drive.getPose());
        flowerTaker.update();

        telemetry.addData("Drive", "robot centric");
        telemetry.addData("Pose reset (DPAD RIGHT)", drive.getPoseReset());
        telemetry.addData("Intake toggle", intakeOn ? "ON" : "OFF");
        telemetry.addData("Flower taker", flowerTaker.isOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Avoidance", slow ? "BYPASSED by slow mode"
                : !RobotConfig.AVOIDANCE_ENABLED ? "OFF"
                : drive.isAvoiding() ? "STEERING" : "enabled");
        drive.addOdometryTelemetry(telemetry);
        stateController.addTelemetry(telemetry, drive.getPose());
        telemetry.update();
    }

    private static double stick(double value) { return ControlMath.deadband(value, RobotConfig.DRIVE_DEADBAND); }

    private boolean held(int driver, Button button) { return ControlBindings.held(driver, button, gamepad1, gamepad2); }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (stateController != null) stateController.stop();
    }
}
