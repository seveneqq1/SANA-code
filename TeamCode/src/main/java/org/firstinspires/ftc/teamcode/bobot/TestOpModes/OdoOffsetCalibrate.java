package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bobot.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Diagnostics;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.PointTrack;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Find Pinpoint offsets by starting still, then making several slow turns in place. */
@NextTeleop(name = "Odo Offset Calibrate", group = "Regional A")
public class OdoOffsetCalibrate extends CommandOpMode {
    private GoBildaPinpointDriver pinpoint;
    private MecanumDrive drive;
    private double startX;
    private double startY;
    private double lastHeading;
    private double totalTurn;

    @Override
    protected void onInit() {
        Motor frontLeft = new Motor(hardwareMap, Constants.FRONT_LEFT);
        Motor frontRight = new Motor(hardwareMap, Constants.FRONT_RIGHT);
        Motor backLeft = new Motor(hardwareMap, Constants.BACK_LEFT);
        Motor backRight = new Motor(hardwareMap, Constants.BACK_RIGHT);
        frontRight.setInverted(true);
        backRight.setInverted(true);
        drive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_NAME);
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PointTrack.TICKS_PER_MM, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        robotTelemetry.addLine("Keep the robot still, then press START.");
        robotTelemetry.update();
    }

    @Override
    protected void onStart() {
        restart();
    }

    @Override
    protected Command createCommand() {
        return Command.build()
                .setExecute(this::measureOffsets)
                .requiring(drive, pinpoint);
    }

    private void restart() {
        pinpoint.update();
        startX = podX();
        startY = podY();
        lastHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        totalTurn = 0;
    }

    // Encoder distance in mm, positive forward (X pod) or left (Y pod).
    private double podX() {
        return (PointTrack.REVERSE_X_POD ? -1 : 1)
                * pinpoint.getEncoderX() / PointTrack.TICKS_PER_MM;
    }

    private double podY() {
        return (PointTrack.REVERSE_Y_POD ? -1 : 1)
                * pinpoint.getEncoderY() / PointTrack.TICKS_PER_MM;
    }

    private void measureOffsets() {
        pinpoint.update();
        if (gamepad1.cross) {
            restart();
        }
        drive.driveRobotCentric(0, 0, gamepad1.right_stick_x * Diagnostics.ODOMETRY_TURN_POWER);

        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        totalTurn += Math.atan2(Math.sin(heading - lastHeading), Math.cos(heading - lastHeading));
        lastHeading = heading;

        double deltaX = podX() - startX;
        double deltaY = podY() - startY;
        robotTelemetry.addData("Turns done", "%.2f  (need 3+)", totalTurn / (2 * Math.PI));
        if (Math.abs(totalTurn) > Math.PI) {
            robotTelemetry.addData("X_POD_OFFSET_MM", "%.1f", -deltaX / totalTurn);
            robotTelemetry.addData("Y_POD_OFFSET_MM", "%.1f", deltaY / totalTurn);
        } else {
            robotTelemetry.addLine("Turn the robot in place with the right stick...");
        }
        robotTelemetry.addData("raw X pod mm", "%.1f", deltaX);
        robotTelemetry.addData("raw Y pod mm", "%.1f", deltaY);
        robotTelemetry.addLine("Cross = start over");
        robotTelemetry.update();
    }

    @Override
    protected void onStop() {
        if (drive != null) {
            drive.stop();
        }
    }
}
