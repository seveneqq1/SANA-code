package org.firstinspires.ftc.teamcode.bobot.controllers;

import static org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new.BlockStates.BLOCK;
import static org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController_new.BlockStates.UNBLOCK;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class StateController1_without_limelight_new {

     
     
     
    public static double SPOOLUP_TIMEOUT_MS = 0.0;

     
     
     
     
    public static double PULSE_OPEN_DURATION_MS = 130.0;
     
     
     
    public static double PULSE_RECOVER_TIMEOUT_MS = 400.0;
     
     
    public static int MAX_BURST_ARTIFACTS = 4;

    public static double FIELD_CENTER_X = 70.75;  
    public static double FIELD_CENTER_Y = 70.75;  

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

    private enum ShootStep { WAIT_SPOOL, PULSE_OPEN, PULSE_RECOVER }
    private ShootStep shootStep = ShootStep.PULSE_OPEN;
    private int artifactsFired = 0;

    public enum State {
        ZERO, INTAKING, PUSH_BALLS_OUT, PREPARING_1, PREPARING_2, SHOOT_1, SHOOT_2
    }

    private State state = State.ZERO;

    public StateController1_without_limelight_new(Follower follower,
                                                  ShooterController_new shooterController,
                                                  IntakeController_new intakeController,
                                                  TransferController_new transferController,
                                                  boolean red) {
        this(follower, shooterController, intakeController, transferController, red, null);
    }

    public StateController1_without_limelight_new(Follower follower,
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
            intakeController.setState(IntakeController_new.State.ZERO);
        }
        if (transferController != null) {
            transferController.setState(TransferController_new.State.ZERO);
        }
        shooting = false;
        artifactsFired = 0;
        setState(State.ZERO);
    }

     
    public void stop_intake() {
        if (shooting) {
            blockBoth();
            shooting = false;
            artifactsFired = 0;
        }
        if (intakeController != null) {
            intakeController.setState(IntakeController_new.State.ZERO);
        }
        if (transferController != null) {
            transferController.setState(TransferController_new.State.ZERO);
        }
        if (state == State.INTAKING || state == State.PUSH_BALLS_OUT
                || state == State.SHOOT_1 || state == State.SHOOT_2) {
            setState(State.ZERO);
        }
    }

    public void intake() {
        if (shooting) return;  
        if (intakeController != null) {
            intakeController.setState(IntakeController_new.State.INTAKING_FULL_POWER);
            transferController.setState(TransferController_new.State.SLOW);
        }
        setState(State.INTAKING);
    }



    public void balls_out() {
        if (shooting) return;
        if (intakeController != null) {
            intakeController.setState(IntakeController_new.State.PUSH_BALLS_OUT);
        }
        if (transferController != null) {
            transferController.setState(TransferController_new.State.PUSH_BALLS_OUT);
        }
        setState(State.PUSH_BALLS_OUT);
    }

     



    public void spinUpBoth() {
        if (shooterController == null) return;
        if (shooterController.getTargetVelocityRPM1() != ShooterController_new.SHOOTER1_RPM) {
            shooterController.setTargetVelocityRPM1(ShooterController_new.SHOOTER1_RPM);
        }
        if (shooterController.getTargetVelocityRPM2() != ShooterController_new.SHOOTER2_RPM) {
            shooterController.setTargetVelocityRPM2(ShooterController_new.SHOOTER2_RPM);
            shooterController.setHoodPosition2(ShooterController_new.HOOD2_POS);
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
        if (shooting) return;  

        spinUpBoth();
        shooting = true;
        artifactsFired = 0;
        setState(shotState);

        if (SPOOLUP_TIMEOUT_MS > 0) {
            shootStep = ShootStep.WAIT_SPOOL;
            timer.reset();
        } else {
            beginBurst(shooterNum);
        }
    }

     




    private void beginBurst(int shooterNum) {
        if (intakeController != null) intakeController.setState(IntakeController_new.State.SUPPLYING);
        if (transferController != null) transferController.setState(TransferController_new.State.SUPPLYING);

        openStopper(shooterNum);
        shootStep = ShootStep.PULSE_OPEN;
        timer.reset();
    }

    private void openStopper(int shooterNum) {
        if (shooterController == null) return;
        if (shooterNum == 1) shooterController.setBlockState1(UNBLOCK);
        else shooterController.setBlockState2(UNBLOCK);
    }

    private void closeStopper(int shooterNum) {
        if (shooterController == null) return;
        if (shooterNum == 1) shooterController.setBlockState1(BLOCK);
        else shooterController.setBlockState2(BLOCK);
    }
    public void prepareShooter1() {
        spinUpBoth();
        if (!shooting) setState(State.PREPARING_1);
    }

    public void prepareShooter2() {
        spinUpBoth();
        if (!shooting) setState(State.PREPARING_2);
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

     
    public int getArtifactsFired() {
        return artifactsFired;
    }

     
     
     

    public void update() {
        if (follower != null) {
            follower.update();
            Pose currentPose = follower.pose();
            if (currentPose != null) {
                x_current = currentPose.x();
                y_current = currentPose.y();
                double headingDeg = Math.toDegrees(currentPose.heading());
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
            if (y_current <= FIELD_CENTER_Y) {
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
            case INTAKING:
            case PUSH_BALLS_OUT:
                shooting = false;
                break;

            case PREPARING_1:
            case PREPARING_2:
                break;

            case SHOOT_1:
                handleShootingSequence(1);
                break;

            case SHOOT_2:
                handleShootingSequence(2);
                break;
        }
    }

    private void handleShootingSequence(int shooterNum) {
        switch (shootStep) {
            case WAIT_SPOOL: {
                boolean atTarget = shooterController != null
                        && (shooterNum == 1
                        ? shooterController.isShooter1AtTarget()
                        : shooterController.isShooter2AtTarget());
                if (atTarget || timer.milliseconds() > SPOOLUP_TIMEOUT_MS) {
                    beginBurst(shooterNum);
                }
                break;
            }

            case PULSE_OPEN:
                if (timer.milliseconds() > PULSE_OPEN_DURATION_MS) {
                    closeStopper(shooterNum);
                    artifactsFired++;
                    timer.reset();

                    if (artifactsFired >= MAX_BURST_ARTIFACTS) {
                        endBurst();
                    } else {
                        shootStep = ShootStep.PULSE_RECOVER;
                    }
                }
                break;

            case PULSE_RECOVER: {
                boolean recovered = shooterController != null
                        && (shooterNum == 1
                        ? shooterController.isShooter1AtTarget()
                        : shooterController.isShooter2AtTarget());
                if (recovered || timer.milliseconds() > PULSE_RECOVER_TIMEOUT_MS) {
                    openStopper(shooterNum);
                    shootStep = ShootStep.PULSE_OPEN;
                    timer.reset();
                }
                break;
            }
        }
    }

    private void endBurst() {
        if (intakeController != null) intakeController.setState(IntakeController_new.State.ZERO);
        if (transferController != null) transferController.setState(TransferController_new.State.ZERO);

        shooting = false;
        setState(State.ZERO);
    }
}