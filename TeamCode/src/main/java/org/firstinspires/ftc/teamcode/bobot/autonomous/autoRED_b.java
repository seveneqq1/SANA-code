package org.firstinspires.ftc.teamcode.bobot.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new;
import org.firstinspires.ftc.teamcode.bobot.controllers.StateController1_without_limelight_new_auto;
import org.firstinspires.ftc.teamcode.bobot.controllers.TransferController_new;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "autored_b")
public class autoRED_b extends OpMode {

     
     
     

    private Follower follower;

    private IntakeController_new intakeController;
    private TransferController_new transferController;
    private ShooterController_new shooterController;

    private StateController1_without_limelight_new_auto stateController;


     
     
     

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorY(70.75);


     
     
     

    private final Pose start = poseFactory.of(10.9204, 85.6372, 180);
    private final Pose path1Start = poseFactory.of(10.9204, 85.6372, 180);
    private final Pose path1 = poseFactory.of(16.2423, 80.4535, 90);
    private final Pose point2 = poseFactory.of(100, 12.5, 30.0968);


     
     
     

    public Path path1() {
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).tangent();
    }

     
     
     

     






    private Command action(Runnable action) {
        return instant(action);
    }

    private Command waitForShot() {
        return Command.build()
                .setDone(() -> !stateController.shooting);
    }


     
     
     

     











     
     
     

    private Command autoRoutine() {

        return sequential(
                action(() -> follower.setPose(start) ),



                 
                 
                 

                 




                parallel(
                        follow(follower, path1()),
                        action(() ->
                                stateController.prepareShooter1()
                        )
                ),



                 
                 
                 

                action(() ->
                        stateController.shooting1()
                ),
                waitForShot(),

                 
                 
                 


                 
                 
                 

                 






                follow(follower, path2()),


                 
                 
                 

                action(() ->
                        stateController.stop_everything()
                )
        );
    }


     
     
     

    @Override
    public void init() {

        Scheduler.reset();


         
         
         

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        follower.update();


         
         
         

        shooterController =
                new ShooterController_new(hardwareMap);

        intakeController =
                new IntakeController_new(hardwareMap);

        transferController =
                new TransferController_new(hardwareMap);


         
         
         

        stateController =
                new StateController1_without_limelight_new_auto(
                        follower,
                        shooterController,
                        intakeController,
                        transferController,
                        false,       
                        telemetry
                );


         
         
         

        stateController.stop_everything();
    }


     
     
     

    @Override
    public void start() {

         



        stateController.stop_everything();

         


        schedule(autoRoutine());
    }


     
     
     

    @Override
    public void loop() {

         
         
         

        follower.update();


         
         
         

         
















        stateController.update();


         
         
         

        Scheduler.execute();


         
         
         

        telemetry.addData(
                "State",
                stateController.getState()
        );

        telemetry.addData(
                "Shooting",
                stateController.shooting
        );

        telemetry.addData(
                "X",
                "%.2f",
                follower.pose().x()
        );

        telemetry.addData(
                "Y",
                "%.2f",
                follower.pose().y()
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                Math.toDegrees(
                        follower.pose().heading()
                )
        );

        telemetry.update();
    }


     
     
     

    @Override
    public void stop() {

         


        if (stateController != null) {
            stateController.stop_everything();
        }

         


        Scheduler.reset();
    }
}