package org.firstinspires.ftc.teamcode.bobot.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

 
import org.firstinspires.ftc.teamcode.pedro.Constants;

 
 
 
 
 
@Autonomous(name = "SANA path Ivy Auto")
@Disabled
public class test extends OpMode {
    private Follower follower;
    private Command auto;
     
     
     
    private static final boolean MIRROR_FOR_OTHER_ALLIANCE = false;
    private final PoseFactory p = MIRROR_FOR_OTHER_ALLIANCE
            ? PoseFactory.degrees().mirrorY(72).mapHeading(h -> -h)
            : PoseFactory.degrees();
    private final Pose startPose = p.of(18, 18, 90);
    private final Pose pose1 = p.of(122, 119, 0);

     
    private final Object shooterResource = new Object();
    private final Object intakeResource = new Object();
    private final Object feedResource = new Object();
    private final Object turretResource = new Object();
    private final boolean routeClearsWalls = true;  
    private final boolean routeClearsSupports = true;  

    @Override
    public void init() {
        Scheduler.reset();
        if (!routeClearsWalls) throw new IllegalArgumentException("Recheck wall clearance after changing the robot footprint.");
        if (!routeClearsSupports) throw new IllegalArgumentException("Recheck hive support clearance against your robot geometry.");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
         
        auto = sequential(
                followPath(path0()),
                instant(this::stopAllSubsystems)
        );
    }

    @Override
    public void start() {
        Scheduler.schedule(auto);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        telemetry.addData("Auto running", Scheduler.isScheduled(auto));
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();  
        if (follower != null) follower.stop();
        stopAllSubsystems();
    }

    private Command followPath(Path route) {
        return follow(follower, route)
                 
                .setDone(() -> !follower.following())
                .requiring(follower);
    }

    private Path path0() {
        return Paths.path(
                Paths.curve(startPose,
                        p.of(56.306, 52.141, 0),
                        p.of(-34.715, 139.28, 0),
                        p.of(98.92, 117.63, 0),
                        pose1).linear(startPose, pose1)
        );
    }


    private void stopAllSubsystems() {
         
    }
}
