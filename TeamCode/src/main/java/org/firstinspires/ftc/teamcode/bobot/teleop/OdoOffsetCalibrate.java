package org.firstinspires.ftc.teamcode.bobot.teleop;

/**
 * Finds the Pinpoint pod offsets automatically.
 * 1) INIT with the robot standing still.  2) START.
 * 3) Turn the robot IN PLACE with the right stick, 3-5 full turns (slowly).
 * 4) Read X_POD_OFFSET_MM / Y_POD_OFFSET_MM on the screen. Cross = start over.
 * Uses TICKS_PER_MM and REVERSE_X/Y_POD from RegionalAPointTrackTeleOp.
 */

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Odo Offset Calibrate", group = "Regional A")
public class OdoOffsetCalibrate extends OpMode {
    private GoBildaPinpointDriver pinpoint;
    private MecanumDrive drive;
    private double startX, startY, lastHeading, totalTurn;

    @Override
    public void init() {
        Motor fl = new Motor(hardwareMap, "frontleft");
        Motor fr = new Motor(hardwareMap, "frontright");
        Motor bl = new Motor(hardwareMap, "backleft");
        Motor br = new Motor(hardwareMap, "backright");
        fr.setInverted(true);
        br.setInverted(true);
        drive = new MecanumDrive(false, fl, fr, bl, br);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(RegionalAPointTrackTeleOp.TICKS_PER_MM, DistanceUnit.MM);
        pinpoint.resetPosAndIMU(); // keep the robot still
        telemetry.addLine("Keep the robot still, then press START.");
        telemetry.update();
    }

    @Override
    public void start() { restart(); }

    private void restart() {
        pinpoint.update();
        startX = podX();
        startY = podY();
        lastHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        totalTurn = 0;
    }

    // Encoder distance in mm, positive = forward (X pod) / left (Y pod).
    private double podX() {
        return (RegionalAPointTrackTeleOp.REVERSE_X_POD ? -1 : 1)
                * pinpoint.getEncoderX() / RegionalAPointTrackTeleOp.TICKS_PER_MM;
    }
    private double podY() {
        return (RegionalAPointTrackTeleOp.REVERSE_Y_POD ? -1 : 1)
                * pinpoint.getEncoderY() / RegionalAPointTrackTeleOp.TICKS_PER_MM;
    }

    @Override
    public void loop() {
        pinpoint.update();
        if (gamepad1.cross) restart();

        drive.driveRobotCentric(0, 0, gamepad1.right_stick_x * 0.4); // turn only

        double h = pinpoint.getHeading(AngleUnit.RADIANS);
        totalTurn += Math.atan2(Math.sin(h - lastHeading), Math.cos(h - lastHeading));
        lastHeading = h;

        double dX = podX() - startX, dY = podY() - startY;
        telemetry.addData("Turns done", "%.2f  (need 3+)", totalTurn / (2 * Math.PI));
        if (Math.abs(totalTurn) > Math.PI) {
            telemetry.addData("X_POD_OFFSET_MM", "%.1f", -dX / totalTurn);
            telemetry.addData("Y_POD_OFFSET_MM", "%.1f", dY / totalTurn);
        } else {
            telemetry.addLine("Turn the robot in place with the right stick...");
        }
        telemetry.addData("raw X pod mm", "%.1f", dX);
        telemetry.addData("raw Y pod mm", "%.1f", dY);
        telemetry.addLine("Cross = start over");
        telemetry.update();
    }

    @Override
    public void stop() { drive.stop(); }
}
