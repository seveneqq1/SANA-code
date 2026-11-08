package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.bobot.controller.shooterController_var2;
import org.firstinspires.ftc.teamcode.bobot.controller.intakeController;

@TeleOp(name = "shoot seq 0.7 +intake+trans")
public class servo3 extends LinearOpMode {
    shooterController_var2 shooter;
    DcMotor intake, transfer;
    boolean intake_on = false;
    public void runOpMode(){
        shooter = new shooterController_var2(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "trans");
        try {
            waitForStart();
            while(opModeIsActive()){
                shooter.setTargetVelocity(shooterController_var2.baseVelocity);
                shooter.update();
                boolean feeding = gamepad1.x && shooter.isAtTarget();
                if(gamepad1.a){
                    intake_on = !intake_on;
                }
                if(intake_on){
                    intake.setPower(1);
                    transfer.setPower(feeding ? -0.8 * intakeController.TRANSFER_SHOOTING_SPEED_SCALE : -1);
                } else {
                    intake.setPower(0);
                    transfer.setPower(0);
                }
                if (feeding) shooter.feedStopper();
                else shooter.closeStopper();
                shooter.debug(telemetry);
                telemetry.update();
            }
        } finally {
            shooter.forceStopFlywheels();
            intake.setPower(0);
            transfer.setPower(0);
        }
    }
}
