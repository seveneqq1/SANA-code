package org.firstinspires.ftc.teamcode.bobot.utlities.debug;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="Shooter_Debug",group = "z")
@Configurable
public class Shooter_Debug extends LinearOpMode {
   public static double  motor_1_power=0, motor_2_power=0, transfer_power=0,intake_power=0;
    private Motor shooter_1,shooter_2,transfer,intake;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
       shooter_1=new Motor(hardwareMap,"shooter1");
       shooter_2=new Motor(hardwareMap,"shooter2");
       transfer=new Motor(hardwareMap,"transfer");
       intake=new Motor(hardwareMap,"intake");
         
        waitForStart();



        while (opModeIsActive()) {
         shooter_1.set(motor_1_power);
         shooter_2.set(motor_2_power);
         intake.set(intake_power);
         transfer.set(transfer_power);
         telemetry.addData("current Velocity shooter1",shooter_1.getCorrectedVelocity());
         telemetry.addData("current Velocity shooter2",  shooter_2.getCorrectedVelocity());
         telemetry.addData("Shooter AMPS 1",((DcMotorEx) shooter_1.motor).getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Shooter AMPS 2",((DcMotorEx) shooter_2.motor).getCurrent(CurrentUnit.MILLIAMPS));
         telemetry.update();

                    }

    }
}