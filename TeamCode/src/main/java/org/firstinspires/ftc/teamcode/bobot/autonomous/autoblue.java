package org.firstinspires.ftc.teamcode.bobot.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import static com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController1_without_limelight_new_auto;
import org.firstinspires.ftc.teamcode.bobot.controllers.TransferController_new;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "Autoblue", group = "Autonomous")
public class autoblue extends LinearOpMode {

    private Timer opmodeTimer;
    private Follower follower;

    private IntakeController_new intakeController;
    private ShooterController_new shooterController;
    private TransferController_new transferController;
    private StateController1_without_limelight_new_auto stateController;
    private boolean preloadShotStarted = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

     
    private final Pose start = poseFactory.of(9.2, 78.8, -90);
    private final Pose path1Start = poseFactory.of(9.2, 78.8, -90);
    private final Pose path1 = poseFactory.of(9.2, 106.5, 90);
    private final Pose path1Control1 = poseFactory.of(17.6, 91.8, 0);
    private final Pose point2 = poseFactory.of(9.2, 131.4, 90);
    private final Pose point3 = poseFactory.of(131.8, 78.8, 90);
    private final Pose point3Control1 = poseFactory.of(86.5, 125.5, 0);
    private final Pose point3Control2 = poseFactory.of(102, 125, 0);
    private final Pose point3Control3 = poseFactory.of(107, 123.4, 0);
    private final Pose point3Control4 = poseFactory.of(109.2, 75.8, 0);
    private final Pose point3Segment1Start = poseFactory.of(131.8, 78.8, 90);
    private final Pose point3Segment1End = poseFactory.of(131.8, 78.8, -30);
    private final Pose point3Segment2Start = poseFactory.of(131.8, 78.8, -30);
    private final Pose point3Segment2End = poseFactory.of(131.8, 78.8, 90);
    private final Pose point4 = poseFactory.of(120.6, 83.4, 0);
    private final Pose point5 = poseFactory.of(130, 83.4, 0);
    private final Pose point6 = poseFactory.of(120.6, 92, -248);
    private final Pose point7 = poseFactory.of(117.5, 132, -270);

    public enum AutoState {
        SHOOT_PRELOAD,
        DRIVE_TO_GARDEN,
        INTAKE_GARDEN,
        DRIVE_TO_GARDEN_SHOOT,
        SHOOT_GARDEN,
        DRIVE_TO_FLOWER,
        INTAKE_FLOWER,
        BACK_AWAY_FLOWER,
        SHOOT_FLOWER,
        PARK,
        IDLE
    }

    private AutoState currentState = AutoState.SHOOT_PRELOAD;
    private ElapsedTime matchTimer;
    private List<LynxModule> allHubs;

    private boolean isShootEnded() {
        return stateController.getState() == StateController1_without_limelight_new_auto.State.FINISHED;
    }

    public void runAutoStateMachine() {
        switch (currentState) {
            case SHOOT_PRELOAD:

                 
                if (!preloadShotStarted) {
                    preloadShotStarted = true;
                    stateController.shooting1();
                }

                 
                if (preloadShotStarted && !stateController.shooting) {

                    stateController.intake();
                    follower.follow(go_to_garden());

                    currentState = AutoState.DRIVE_TO_GARDEN;
                }

                break;

            case DRIVE_TO_GARDEN:
                if (!follower.isBusy()) {
                    follower.follow(intake_garden());
                    currentState = AutoState.INTAKE_GARDEN;
                }
                break;

            case INTAKE_GARDEN:
                if (!follower.isBusy()) {
                    follower.follow(garden_shoot());
                    currentState = AutoState.DRIVE_TO_GARDEN_SHOOT;
                }
                break;

            case DRIVE_TO_GARDEN_SHOOT:
                if (!follower.isBusy()) {
                    stateController.shooting1();
                    currentState = AutoState.SHOOT_GARDEN;
                }
                break;

            case SHOOT_GARDEN:
                if (isShootEnded()) {
                     
                    stateController.intake();
                    follower.follow(go_to_flower());
                    currentState = AutoState.DRIVE_TO_FLOWER;
                }
                break;

            case DRIVE_TO_FLOWER:
                if (!follower.isBusy()) {
                    follower.follow(flower_intake());
                    currentState = AutoState.INTAKE_FLOWER;
                }
                break;

            case INTAKE_FLOWER:
                if (!follower.isBusy()) {
                    follower.follow(flower_shoot());
                    currentState = AutoState.BACK_AWAY_FLOWER;
                }
                break;

            case BACK_AWAY_FLOWER:
                if (!follower.isBusy()) {
                    stateController.shooting1();
                    currentState = AutoState.SHOOT_FLOWER;
                }
                break;

            case SHOOT_FLOWER:
                if (isShootEnded()) {
                    follower.follow(park());
                    currentState = AutoState.PARK;
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    currentState = AutoState.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        matchTimer = new ElapsedTime();
        opmodeTimer = new Timer();
        opmodeTimer.reset();

        intakeController = new IntakeController_new(hardwareMap);
        shooterController = new ShooterController_new(hardwareMap);
        transferController = new TransferController_new(hardwareMap);

        stateController = new StateController1_without_limelight_new_auto(
                follower,
                shooterController,
                intakeController,
                transferController,
                false
        );

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (!opModeIsActive() && !isStopRequested()) {
            clearHubCaches();
            follower.update();
        }

        if (isStopRequested()) return;

        matchTimer.reset();

        while (opModeIsActive()) {
            clearHubCaches();

            runAutoStateMachine();

            stateController.update();
            follower.update();

            if (matchTimer.milliseconds() > 31200) {
                requestOpModeStop();
            }

            telemetry.addData("Auto State", currentState);
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Distance Remaining", follower.distanceToEndpoint());
            }

            telemetry.update();
        }
    }

    private void clearHubCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public Path go_to_garden() {
        return curve(path1Start, path1Control1, path1).linear(path1Start, path1);
    }

    public Path intake_garden() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path garden_shoot() {
        return curve(point2, point3Control1, point3Control2, point3Control3, point3Control4, point3).heading(
                Interpolator.piecewise()
                        .until(0.4982, Interpolator.linear(point3Segment1Start, point3Segment1End))
                        .until(1.0000, Interpolator.linear(point3Segment2Start, point3Segment2End))
        );
    }

    public Path go_to_flower() {
        return line(point3, point4).linear(point3, point4);
    }

    public Path flower_intake() {
        return line(point4, point5).linear(point4, point5);
    }

    public Path flower_shoot() {
        return line(point5, point6).linear(point5, point6);
    }

    public Path park() {
        return line(point6, point7).linear(point6, point7);
    }
}