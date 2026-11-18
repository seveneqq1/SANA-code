package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.teleop.comp.trackingTeleOp;

@TeleOp(name = "Beka tracking test", group = "Tests")
public class TrackingTestTeleOp extends trackingTeleOp {
    @Override
    protected void onStart() {
        r.drive.resetPos();
        super.onStart();
    }
}
