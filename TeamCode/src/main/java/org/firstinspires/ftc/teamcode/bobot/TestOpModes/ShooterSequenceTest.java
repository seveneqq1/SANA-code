package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.bobot.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.bobot.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Diagnostics;

@NextTeleop(name = "shoot seq 0.7 +intake+trans", group = OpModeMeta.DefaultGroup)
public class ShooterSequenceTest extends CommandOpMode {
    private ShooterController shooter;
    private DcMotor intake;
    private DcMotor transfer;
    private boolean intakeOn;

    @Override
    protected void onInit() {
        shooter = new ShooterController(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "trans");
    }

    @Override
    protected Command createCommand() {
        return Command.build()
                .setExecute(this::runShooterSequence)
                .requiring(shooter, intake, transfer);
    }

    private void runShooterSequence() {
        shooter.setTargetVelocity(ShooterController.baseVelocity);
        shooter.update();
        boolean feeding = gamepad1.x && shooter.isAtTarget();
        // This test toggles once per control loop while A is held.
        if (gamepad1.a) {
            intakeOn = !intakeOn;
        }
        if (intakeOn) {
            intake.setPower(1);
            transfer.setPower(feeding
                    ? Diagnostics.TRANSFER_TEST_FEED_POWER * IntakeController.TRANSFER_SHOOTING_SPEED_SCALE
                    : -1);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }
        if (feeding) {
            shooter.feedStopper();
        } else {
            shooter.closeStopper();
        }
        shooter.debug(robotTelemetry);
        robotTelemetry.update();
    }

    @Override
    protected void onStop() {
        if (shooter != null) {
            shooter.forceStopFlywheels();
        }
        if (intake != null) {
            intake.setPower(0);
        }
        if (transfer != null) {
            transfer.setPower(0);
        }
    }
}
