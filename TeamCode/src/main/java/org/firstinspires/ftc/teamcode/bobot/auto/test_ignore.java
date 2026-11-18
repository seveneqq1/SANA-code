package org.firstinspires.ftc.teamcode.bobot.auto;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class test_ignore extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.1446, 7.8917, 180);
    private final Pose togarden = poseFactory.of(9, 7.9, 180);
    private final Pose shoot1Start = poseFactory.of(9, 7.9, 180);
    private final Pose shoot1 = poseFactory.of(46.5, 120, 90);
    private final Pose shoot1Control1 = poseFactory.of(30, 110, 0);
    private final Pose flowerStart = poseFactory.of(46.5, 120, 90);
    private final Pose flower = poseFactory.of(46.5, 132, 90);
    private final Pose parkStart = poseFactory.of(46.5, 132, 90);
    private final Pose park = poseFactory.of(7.5, 120, 0);
    private final Pose parkControl1 = poseFactory.of(40, 116, 0);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, togarden()),
                new FollowPathCommand(follower, shoot1()),
                new FollowPathCommand(follower, flower()),
                new FollowPathCommand(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        if (follower == null) {
            telemetry.addLine("bruhhh tune constants in pedro");
            telemetry.update();
            return;
        }
        follower.setPose(start);
        follower.update();

        waitForStart();
        if (isStopRequested()) return;

        CommandScheduler scheduler = CommandScheduler.getInstance();
        SequentialCommandGroup routine = autoRoutine();
        try {
            scheduler.schedule(routine);
            while (opModeIsActive()) {
                follower.update();
                scheduler.run();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());

                telemetry.update();
            }
        } finally {
            scheduler.cancel(routine);
            follower.stop();
            follower.update();
        }
    }

    public Path togarden() {
        return Paths.line(start, togarden).linear(start, togarden);
    }

    public Path shoot1() {
        return Paths.curve(shoot1Start, shoot1Control1, shoot1).linear(shoot1Start, shoot1);
    }

    public Path flower() {
        return Paths.line(flowerStart, flower).linear(flowerStart, flower);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
