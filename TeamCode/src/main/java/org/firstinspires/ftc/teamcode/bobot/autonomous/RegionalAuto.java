package org.firstinspires.ftc.teamcode.bobot.autonomous;

import org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.closeFlowerTaker;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.intake;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.openFlowerTaker;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.shoot;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.spinUp;
import static org.firstinspires.ftc.teamcode.bobot.autonomous.commands.RobotCommands.waitUntilReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.bobot.autonomous.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;
import org.firstinspires.ftc.teamcode.bobot.controllers.FlowerTakerController;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;
import org.firstinspires.ftc.teamcode.bobot.states.PoseStorage;

public abstract class RegionalAuto extends OpMode {
    protected Follower follower;
    protected StateController stateController;
    protected FlowerTakerController flowerTaker;
    protected AutoPaths paths;
    private Command routine;

    protected abstract Alliance alliance();

    protected Command routine() {
        return sequential(
                waitUntilReady(stateController),
                shoot(stateController, RobotConfig.AUTO_FEED_MS),

                intake(stateController),
                follow(follower, paths.path1()),
                spinUp(stateController),
                follow(follower, paths.path2()),
                waitUntilReady(stateController),
                shoot(stateController, RobotConfig.AUTO_FEED_MS),

                follow(follower, paths.path3()),
                openFlowerTaker(flowerTaker),
                follow(follower, paths.path4()),
                parallel(follow(follower, paths.path5()), spinUp(stateController)),
                waitUntilReady(stateController),
                shoot(stateController, RobotConfig.AUTO_FEED_MS),
                follow(follower, paths.path6())
        );
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
        PoseStorage.clear();
        follower = Constants.create(hardwareMap);
        paths = new AutoPaths(alliance());
        follower.setPose(paths.start);
        stateController = new StateController(hardwareMap, alliance());
        flowerTaker = new FlowerTakerController(hardwareMap);
        routine = routine();
        telemetry.addLine("Place the robot on the start pose shown below.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        stateController.stop();
        telemetry.addData("Start pose", follower.pose());
        telemetry.update();
    }

    @Override
    public void start() { Scheduler.schedule(routine); }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        stateController.update(follower.pose());
        flowerTaker.update();
        PoseStorage.save(follower.pose(), alliance());

        telemetry.addData("Routine", routine.isScheduled() ? "running" : "finished");
        stateController.addTelemetry(telemetry, follower.pose());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (stateController != null) stateController.stop();
        if (follower != null) follower.stop();
    }
}
