package org.firstinspires.ftc.teamcode.bobot.controllers;

import static org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new.BlockStates.BLOCK;
import static org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new.BlockStates.UNBLOCK;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateController1_without_limelight_new_auto {

     
     
     

    public static double FEED_DURATION_MS = 900.0;
    public static double SPOOLUP_TIMEOUT_MS = 1200.0;

    public static double FIELD_CENTER_X = 72.0;

     
     
     

    private final Follower follower;
    private final ShooterController_new shooterController;
    private final IntakeController_new intakeController;
    private final TransferController_new transferController;

    private Telemetry telemetry;

     
     
     

    public final ElapsedTime timer;
    private final ElapsedTime stateTimer;

     
     
     

    public boolean isRed;
    public boolean shooting = false;

    private double currentHeading = 0;
    private double x_velocity = 0;
    private double y_velocity = 0;
    private double x_current = 0;
    private double y_current = 0;

     
     
     

    private boolean leftSide = true;
    private boolean rightSide = false;
    private boolean sideManualOverride = false;

     
     
     

    private enum ShootStep {
        WAIT_SPOOL,
        FEEDING,
        FINISHED
    }

    private ShootStep shootStep = ShootStep.WAIT_SPOOL;

     
     
     

    public enum State {
        ZERO,
        INTAKING,
        PUSH_BALLS_OUT,
        PREPARING_1,
        PREPARING_2,
        SHOOT_1,
        SHOOT_2,
        FINISHED
    }

    private State state = State.ZERO;

     
     
     

    public StateController1_without_limelight_new_auto(
            Follower follower,
            ShooterController_new shooterController,
            IntakeController_new intakeController,
            TransferController_new transferController,
            boolean red) {

        this(
                follower,
                shooterController,
                intakeController,
                transferController,
                red,
                null
        );
    }

    public StateController1_without_limelight_new_auto(
            Follower follower,
            ShooterController_new shooterController,
            IntakeController_new intakeController,
            TransferController_new transferController,
            boolean red,
            Telemetry telemetry) {

        this.follower = follower;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.transferController = transferController;

        this.isRed = red;
        this.telemetry = telemetry;

        this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

         
         
        if (this.shooterController != null) {
            this.shooterController.setBlockState1(BLOCK);
            this.shooterController.setBlockState2(BLOCK);
        }
    }

     
     
     

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
        stateTimer.reset();
    }

     
     
     

     









    public void stop_everything() {

        if (shooterController != null) {
            shooterController.stop();

             
            shooterController.setBlockState1(BLOCK);
            shooterController.setBlockState2(BLOCK);
        }

        if (intakeController != null) {
            intakeController.setState(
                    IntakeController_new.State.ZERO
            );
        }

        if (transferController != null) {
            transferController.setState(
                    TransferController_new.State.ZERO
            );
        }

        shooting = false;
        shootStep = ShootStep.FINISHED;

        setState(State.ZERO);
    }

     






    public void stop_intake() {

        if (shooting) {

             
            blockBoth();

            shooting = false;
            shootStep = ShootStep.FINISHED;
        }

        if (intakeController != null) {
            intakeController.setState(
                    IntakeController_new.State.ZERO
            );
        }

        if (transferController != null) {
            transferController.setState(
                    TransferController_new.State.ZERO
            );
        }

        if (state == State.INTAKING
                || state == State.PUSH_BALLS_OUT
                || state == State.SHOOT_1
                || state == State.SHOOT_2) {

            setState(State.ZERO);
        }
    }

     





    public void intake() {

        if (shooting) {
            return;
        }

        if (intakeController != null) {
            intakeController.setState(
                    IntakeController_new.State.INTAKING_FULL_POWER
            );
        }

        if (transferController != null) {
            transferController.setState(
                    TransferController_new.State.SLOW
            );
        }

        setState(State.INTAKING);
    }

     





    public void balls_out() {

        if (shooting) {
            return;
        }

        if (intakeController != null) {
            intakeController.setState(
                    IntakeController_new.State.PUSH_BALLS_OUT
            );
        }

        if (transferController != null) {
            transferController.setState(
                    TransferController_new.State.PUSH_BALLS_OUT
            );
        }

        setState(State.PUSH_BALLS_OUT);
    }

     
     
     

     




    public void spinUpBoth() {

        if (shooterController == null) {
            return;
        }

        if (shooterController.getTargetVelocityRPM1()
                != ShooterController_new.SHOOTER1_RPM) {

            shooterController.setTargetVelocityRPM1(
                    ShooterController_new.SHOOTER1_RPM
            );
        }

        if (shooterController.getTargetVelocityRPM2()
                != ShooterController_new.SHOOTER2_RPM) {

            shooterController.setTargetVelocityRPM2(
                    ShooterController_new.SHOOTER2_RPM
            );

            shooterController.setHoodPosition2(
                    ShooterController_new.HOOD2_POS
            );
        }
    }

     
     
     

     
    public void shootPollen() {
        shooting1();
    }

    public void shooting1() {
        startShot(State.SHOOT_1, 1);
    }

     
    public void shootNectar() {
        shooting2();
    }

    public void shooting2() {
        startShot(State.SHOOT_2, 2);
    }

     





    private void startShot(State shotState, int shooterNum) {

        if (shooting) {
            return;
        }

         
        spinUpBoth();

        shooting = true;
        shootStep = ShootStep.WAIT_SPOOL;

        timer.reset();

        setState(shotState);

         
         
    }

     


    private void beginFeeding(int shooterNum) {

        if (shooterController != null) {

            if (shooterNum == 1) {

                shooterController.setBlockState1(UNBLOCK);

            } else {

                shooterController.setBlockState2(UNBLOCK);
            }
        }

        if (intakeController != null) {
            intakeController.setState(
                    IntakeController_new.State.SUPPLYING
            );
        }

        if (transferController != null) {
            transferController.setState(
                    TransferController_new.State.SUPPLYING
            );
        }

        shootStep = ShootStep.FEEDING;
        timer.reset();
    }

     
     
     

    public void prepareShooter1() {

        spinUpBoth();

        if (!shooting) {
            setState(State.PREPARING_1);
        }
    }

    public void prepareShooter2() {

        spinUpBoth();

        if (!shooting) {
            setState(State.PREPARING_2);
        }
    }

     
     
     

     


    private void blockBoth() {

        if (shooterController != null) {

            shooterController.setBlockState1(BLOCK);
            shooterController.setBlockState2(BLOCK);
        }
    }

     
     
     

    public void setLeftSide() {

        leftSide = true;
        rightSide = false;
        sideManualOverride = true;
    }

    public void setRightSide() {

        rightSide = true;
        leftSide = false;
        sideManualOverride = true;
    }

    public void enableAutoSideDetection() {

        sideManualOverride = false;
    }

    public boolean isLeftSide() {
        return leftSide;
    }

    public boolean isRightSide() {
        return rightSide;
    }

     
     
     

    public double getHeading() {
        return currentHeading;
    }

    public double getX() {
        return x_current;
    }

    public double getY() {
        return y_current;
    }

     
     
     

    public void update() {

         
         
         

        if (follower != null) {

            follower.update();

            Pose currentPose = follower.pose();

            if (currentPose != null) {

                x_current = currentPose.x();
                y_current = currentPose.y();

                double headingDeg =
                        Math.toDegrees(currentPose.heading());

                if (headingDeg < 0) {
                    headingDeg += 360;
                }

                currentHeading = headingDeg;
            }

            if (follower.velocity() != null) {

                x_velocity = follower.velocity().vx;
                y_velocity = follower.velocity().vy;
            }
        }

         
         
         

        if (!sideManualOverride) {

            if (x_current <= FIELD_CENTER_X) {

                leftSide = true;
                rightSide = false;

            } else {

                leftSide = false;
                rightSide = true;
            }
        }

         
         
         

        if (shooterController != null) {
            shooterController.update();
        }

        if (intakeController != null) {
            intakeController.update();
        }

        if (transferController != null) {
            transferController.update();
        }

         
         
         

        switch (state) {

            case ZERO:

                shooting = false;
                break;

            case INTAKING:

                shooting = false;
                break;

            case PUSH_BALLS_OUT:

                shooting = false;
                break;

            case PREPARING_1:

                 
                break;

            case PREPARING_2:

                 
                break;

            case SHOOT_1:

                handleShootingSequence(1);
                break;

            case SHOOT_2:

                handleShootingSequence(2);
                break;

            case FINISHED:

                 
                shooting = false;
                break;
        }
    }

     
     
     

    private void handleShootingSequence(int shooterNum) {

        if (shooterController == null) {
             
             
             
            if (intakeController != null) {
                intakeController.setState(
                        IntakeController_new.State.ZERO
                );
            }

            if (transferController != null) {
                transferController.setState(
                        TransferController_new.State.ZERO
                );
            }

            shooting = false;
            shootStep = ShootStep.FINISHED;
            setState(State.FINISHED);
            return;
        }

        switch (shootStep) {

             
             
             

            case WAIT_SPOOL: {

                boolean atTarget;

                if (shooterNum == 1) {

                    atTarget =
                            shooterController.isShooter1AtTarget();

                } else {

                    atTarget =
                            shooterController.isShooter2AtTarget();
                }

                 






                if (atTarget
                        || timer.milliseconds() > SPOOLUP_TIMEOUT_MS) {

                    beginFeeding(shooterNum);
                }

                break;
            }

             
             
             

            case FEEDING:

                if (timer.milliseconds() > FEED_DURATION_MS) {

                     
                     
                     
                     

                    if (shooterController != null) {

                        if (shooterNum == 1) {

                            shooterController.setBlockState1(BLOCK);

                        } else {

                            shooterController.setBlockState2(BLOCK);
                        }

                         
                         
                        shooterController.stop();
                    }

                     
                    if (intakeController != null) {

                        intakeController.setState(
                                IntakeController_new.State.ZERO
                        );
                    }

                     
                    if (transferController != null) {

                        transferController.setState(
                                TransferController_new.State.ZERO
                        );
                    }

                    shooting = false;

                    shootStep = ShootStep.FINISHED;

                    setState(State.FINISHED);
                }

                break;

             
             
             

            case FINISHED:

                shooting = false;
                setState(State.ZERO);

                break;
        }
    }
}