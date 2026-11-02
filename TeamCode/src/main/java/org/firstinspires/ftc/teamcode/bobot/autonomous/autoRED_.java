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

@Autonomous(name = "autored_")
public class autoRED_ extends OpMode {

     
     
     

    private Follower follower;

    private IntakeController_new intakeController;
    private TransferController_new transferController;
    private ShooterController_new shooterController;

    private StateController1_without_limelight_new_auto stateController;


     
     
     

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);


     
     
     

    private final Pose start =
            poseFactory.of(9.2, 78.8, -90);

    private final Pose path1Start =
            poseFactory.of(9.2, 78.8, -90);

    private final Pose path1 =
            poseFactory.of(9.2, 106.5, 90);

    private final Pose path1Control1 =
            poseFactory.of(17.6, 91.8, 0);

    private final Pose point2 =
            poseFactory.of(9.2, 131.4, 90);

    private final Pose point3 =
            poseFactory.of(131.8, 78.8, 90);

    private final Pose point3Control1 =
            poseFactory.of(86.5, 125.5, 0);

    private final Pose point3Control2 =
            poseFactory.of(102, 125, 0);

    private final Pose point3Control3 =
            poseFactory.of(107, 123.4, 0);

    private final Pose point3Control4 =
            poseFactory.of(109.2, 75.8, 0);

    private final Pose point3Segment1Start =
            poseFactory.of(131.8, 78.8, 90);

    private final Pose point3Segment1End =
            poseFactory.of(131.8, 78.8, -30);

    private final Pose point3Segment2Start =
            poseFactory.of(131.8, 78.8, -30);

    private final Pose point3Segment2End =
            poseFactory.of(131.8, 78.8, 90);

    private final Pose point4 =
            poseFactory.of(120.6, 86.7, 0);

    private final Pose point5 =
            poseFactory.of(130, 86.7, 0);

    private final Pose point6 =
            poseFactory.of(120.6, 92, -248);

    private final Pose point7 =
            poseFactory.of(117.5, 132, -270);


     
     
     

    public Path go_to_garden() {

        return curve(
                path1Start,
                path1Control1,
                path1
        ).linear(
                path1Start,
                path1
        );
    }


    public Path intake_garden() {

        return line(
                path1,
                point2
        ).linear(
                path1,
                point2
        );
    }


    public Path garden_go_shoot() {

        return curve(
                point2,
                point3Control1,
                point3Control2,
                point3Control3,
                point3Control4,
                point3
        ).heading(
                Interpolator.piecewise()
                        .until(
                                0.4982,
                                Interpolator.linear(
                                        point3Segment1Start,
                                        point3Segment1End
                                )
                        )
                        .until(
                                1,
                                Interpolator.linear(
                                        point3Segment2Start,
                                        point3Segment2End
                                )
                        )
        );
    }


    public Path go_flower() {

        return line(
                point3,
                point4
        ).linear(
                point3,
                point4
        );
    }


    public Path take_from_flower() {

        return line(
                point4,
                point5
        ).linear(
                point4,
                point5
        );
    }


    public Path go_back() {

        return line(
                point5,
                point6
        ).linear(
                point5,
                point6
        );
    }


    public Path park() {

        return line(
                point6,
                point7
        ).linear(
                point6,
                point7
        );
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

                 
                 
                 

                 




                action(() ->
                        stateController.prepareShooter1()
                ),


                 










                action(() ->
                        stateController.shooting1()
                ),


                 









                waitForShot(),


                 
                 
                 

                action(() ->
                        stateController.intake()
                ),


                 
                 
                 

                follow(
                        follower,
                        go_to_garden()
                ),


                 
                 
                 

                follow(
                        follower,
                        intake_garden()
                ),


                 
                 
                 

                action(() ->
                        stateController.stop_intake()
                ),


                 
                 
                 

                 






                parallel(

                        follow(
                                follower,
                                garden_go_shoot()
                        ),

                        action(() ->
                                stateController.prepareShooter1()
                        )
                ),


                 
                 
                 

                action(() ->
                        stateController.shooting1()
                ),

                waitForShot(),


                 
                 
                 

                action(() ->
                        stateController.intake()
                ),


                 
                 
                 

                follow(
                        follower,
                        go_flower()
                ),

                action(() ->
                        intakeController.setFlPos(0.2)
                ),

                 
                 
                 

                follow(
                        follower,
                        take_from_flower()
                ),


                 
                 
                 

                action(() ->
                    intakeController.setFlPos(0.9)
                ),

                action(() ->
                        stateController.stop_intake()
                ),


                 
                 
                 

                 






                parallel(

                        follow(
                                follower,
                                go_back()
                        ),

                        action(() ->
                                stateController.prepareShooter1()
                        )
                ),


                 
                 
                 

                action(() ->
                        stateController.shooting1()
                ),

                waitForShot(),


                 
                 
                 

                follow(
                        follower,
                        park()
                ),


                 
                 
                 

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