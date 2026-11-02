package org.firstinspires.ftc.teamcode.bobot.controllers;

import static org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController.BlockStates.BLOCK;
import static org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController.BlockStates.UNBLOCK;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

 








public class StateController1 {

     
    public static double FEED_DURATION_MS = 900.0;
    public static double SPOOLUP_TIMEOUT_MS = 1200.0;
    public static double MAX_RELOCALIZE_VELOCITY = 2.0;  

    private final Limelight3A limelight;
    private final Follower follower;
    private final ShooterController shooterController;
    private final IntakeController intakeController;
    private final TransferController transferController;
    private Telemetry telemetry;

    public final ElapsedTime timer;
    private final ElapsedTime stateTimer;

    public boolean isRed;
    public boolean shooting = false;

     
    private double currentRaw = 0;
    private double currentHeading = 0;
    private double x_velocity = 0;
    private double y_velocity = 0;
    private double x_current = 0;
    private double y_current = 0;

     
    private boolean leftSide = true;
    private boolean rightSide = false;

     
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
        SHOOT_2
    }

    private State state = State.ZERO;

    public StateController1(Limelight3A limelight, Follower follower,
                            ShooterController shooterController,
                            IntakeController intakeController,
                            TransferController transferController,
                            boolean red) {
        this(limelight, follower, shooterController, intakeController, transferController, red, null);
    }

    public StateController1(Limelight3A limelight, Follower follower,
                            ShooterController shooterController,
                            IntakeController intakeController,
                            TransferController transferController,
                            boolean red,
                            Telemetry telemetry) {
        this.limelight = limelight;
        this.follower = follower;
        this.shooterController = shooterController;
        this.intakeController = intakeController;
        this.transferController = transferController;
        this.isRed = red;
        this.telemetry = telemetry;

        this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.stateTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        if (this.limelight != null) {
            this.limelight.pipelineSwitch(0);
            this.limelight.start();
        }

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
            intakeController.setState(IntakeController.State.ZERO);
        }
        if (transferController != null) {
            transferController.setState(TransferController.State.ZERO);
        }
        shooting = false;
        setState(State.ZERO);
    }

    public void stop_intake() {
        if (intakeController != null) {
            intakeController.setState(IntakeController.State.ZERO);
        }
        if (transferController != null) {
            transferController.setState(TransferController.State.ZERO);
        }
        if (state == State.INTAKING || state == State.PUSH_BALLS_OUT) {
            setState(State.ZERO);
        }
    }

    public void intake() {
        if (intakeController != null) {
            intakeController.setState(IntakeController.State.INTAKING_FULL_POWER);
        }
        if (transferController != null) {
            transferController.setState(TransferController.State.TRANSFER_FULL_POWER);
        }
        setState(State.INTAKING);
    }

    public void balls_out() {
        if (intakeController != null) {
            intakeController.setState(IntakeController.State.PUSH_BALLS_OUT);
        }
        if (transferController != null) {
            transferController.setState(TransferController.State.PUSH_BALLS_OUT);
        }
        setState(State.PUSH_BALLS_OUT);
    }

     
    public void shootPollen() {
        shooting1();
    }

    public void shooting1() {
        shooting = true;
        shootStep = ShootStep.WAIT_SPOOL;
        timer.reset();

        if (shooterController != null) {
            shooterController.setTargetVelocityRPM1(ShooterController.SHOOTER1_RPM);
        }
        setState(State.SHOOT_1);
    }

     
    public void shootNectar() {
        shooting2();
    }

    public void shooting2() {
        shooting = true;
        shootStep = ShootStep.WAIT_SPOOL;
        timer.reset();

        if (shooterController != null) {
            shooterController.setTargetVelocityRPM2(ShooterController.SHOOTER2_RPM);
            shooterController.setHoodPosition2(ShooterController.HOOD2_POS);
        }
        setState(State.SHOOT_2);
    }

     
    public void prepareShooter1() {
        if (shooterController != null) {
            shooterController.setTargetVelocityRPM1(ShooterController.SHOOTER1_RPM);
        }
        setState(State.PREPARING_1);
    }

    public void prepareShooter2() {
        if (shooterController != null) {
            shooterController.setTargetVelocityRPM2(ShooterController.SHOOTER2_RPM);
            shooterController.setHoodPosition2(ShooterController.HOOD2_POS);
        }
        setState(State.PREPARING_2);
    }

     

    public void setLeftSide() {
        leftSide = true;
        rightSide = false;
    }

    public void setRightSide() {
        rightSide = true;
        leftSide = false;
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
                currentRaw = currentPose.heading() / (Math.PI * 2) * 360;
                if (currentRaw < 0) {
                    currentRaw += 360;
                }
                currentHeading = currentRaw;
            }
            if (follower.velocity() != null) {
                x_velocity = follower.velocity().vx;
                y_velocity = follower.velocity().vy;
            }
        }

         
        if (x_current <= 72.0) {
            leftSide = true;
            rightSide = false;
        } else {
            leftSide = false;
            rightSide = true;
        }

         
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                updateLimelightRelocalization(result);
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
        boolean atTarget = (shooterNum == 1)
                ? (shooterController != null && shooterController.isShooter1AtTarget())
                : (shooterController != null && shooterController.isShooter2AtTarget());

        switch (shootStep) {
            case WAIT_SPOOL:
                if (atTarget || timer.milliseconds() > SPOOLUP_TIMEOUT_MS) {
                    if (shooterController != null) {
                        if (shooterNum == 1) shooterController.setBlockState1(UNBLOCK);
                        else shooterController.setBlockState2(UNBLOCK);
                    }
                    if (intakeController != null) intakeController.setState(IntakeController.State.SUPPLYING);
                    if (transferController != null) transferController.setState(TransferController.State.SUPPLYING);

                    shootStep = ShootStep.FEEDING;
                    timer.reset();
                }
                break;

            case FEEDING:
                if (timer.milliseconds() > FEED_DURATION_MS) {
                    if (shooterController != null) {
                        if (shooterNum == 1) shooterController.setBlockState1(BLOCK);
                        else shooterController.setBlockState2(BLOCK);
                        shooterController.stop();
                    }
                    if (intakeController != null) intakeController.setState(IntakeController.State.ZERO);
                    if (transferController != null) transferController.setState(TransferController.State.ZERO);

                    shootStep = ShootStep.FINISHED;
                    shooting = false;
                    setState(State.ZERO);
                }
                break;

            case FINISHED:
                shooting = false;
                setState(State.ZERO);
                break;
        }
    }

     



    private void updateLimelightRelocalization(LLResult result) {
        if (follower == null || result.getFiducialResults().isEmpty()) {
            return;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        double xIn = botpose.getPosition().x * 39.3701;
        double yIn = botpose.getPosition().y * 39.3701;
        double hRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

         
        if (xIn >= -72 && xIn <= 72 && yIn >= -72 && yIn <= 72 && (xIn < 0 || yIn < 0)) {
            xIn += 72.0;
            yIn += 72.0;
        }

         
        double robotVel = Math.hypot(x_velocity, y_velocity);
        if (robotVel < MAX_RELOCALIZE_VELOCITY && xIn >= 0 && xIn <= 144 && yIn >= 0 && yIn <= 144) {
            follower.setPose(new Pose(xIn, yIn, hRad));
        }
    }
}