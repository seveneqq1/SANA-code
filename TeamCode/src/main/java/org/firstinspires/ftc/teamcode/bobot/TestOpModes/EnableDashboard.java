package org.firstinspires.ftc.teamcode.bobot.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Method;

@TeleOp(name = "Enable Dashboard")
public class EnableDashboard extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Ready to enable FTC Dashboard");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {
            FtcDashboard dashboard = FtcDashboard.getInstance();

            if (dashboard == null) {
                telemetry.addLine("Dashboard instance is NULL!");
                telemetry.update();
                return;
            }

            Method method = FtcDashboard.class
                    .getDeclaredMethod("enable");

            method.setAccessible(true);
            method.invoke(dashboard);

            telemetry.addData(
                    "Dashboard Enabled",
                    dashboard.isEnabled()
            );

        } catch (Exception e) {
            telemetry.addData("ERROR", e.toString());
        }

        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }
    }
}