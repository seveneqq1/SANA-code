package org.firstinspires.ftc.teamcode.bobot.utlities.debug;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.RGBLight;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.bobot.controllers.TurretController;

import java.util.List;



@Configurable
@TeleOp(group = "debug")
public class DebugAll extends LinearOpMode {
    ShooterController shooterController;
    IntakeController intakeController;
    TurretController turretController;
    RGBLight rgbLight;

    public static double shooter_power=0;
    public static double intake_power=0;
    public static double transfer_power=0;
    public static double ramp_angle=0.5;
    public static double turret_target=0;
    public static double blocker_balls=0.5;
    public static double rgb_position=0.5;
    public Motor shooter_left;
    public Motor shooter_right;
    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intakeController=new IntakeController(hardwareMap);
        shooterController=new ShooterController(hardwareMap);
        rgbLight=new RGBLight(hardwareMap);
        shooter_left=new Motor(hardwareMap,"shooter2");
        shooter_right=new Motor(hardwareMap,"shooter1");
        turretController=new TurretController(hardwareMap);
        VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        double lastTimeStamp = 0;
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;

            double period = currentTimeStamp - lastTimeStamp;
            lastTimeStamp = currentTimeStamp;
            double frequency = 1 / period;
            shooter_left.set(shooter_power);
            shooter_right.set(-shooter_power);
            shooterController.setBlockerPosition(blocker_balls);
            intakeController.setIntakePower(intake_power);
            intakeController.setTransferPower(transfer_power);
            shooterController.setRampPosition(ramp_angle);
            turretController.setTargetPosition(turret_target);
            rgbLight.setRGB_PWM_Signal(rgb_position);
            turretController.update();
            double voltage = voltageSensor.getVoltage();
            telemetry.addData("shooter AMPS", shooterController.getCurrentAMPS());
            telemetry.addData("corrected_velocity turret",shooterController.getCorrectedVelocityOfTurret());
            telemetry.addData("turret target position", turret_target);
            telemetry.addData("Transfer AMPS", intakeController.getMILLIAMPS_transfer());
            telemetry.addData("Transfer Velocity", intakeController.getTransferVelocity());
            telemetry.addData("Intake Velocity",intakeController.getIntakeVelocity());
            telemetry.addData("Intake AMPS",intakeController.getMILLIAMPS_intake());
            telemetry.addData(" current velocity",(shooter_left.getCorrectedVelocity()+shooter_right.getCorrectedVelocity()));
            telemetry.addData("frequency", frequency);
            telemetry.addData("voltage", voltage);
            telemetry.update();
        }

    }
}