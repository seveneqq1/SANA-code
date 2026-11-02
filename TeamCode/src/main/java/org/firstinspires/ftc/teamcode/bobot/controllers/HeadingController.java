package org.firstinspires.ftc.teamcode.bobot.controllers;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bobot.utlities.trashs.LerpController;


@Configurable
public class HeadingController {

    public static double LERP_RATE = 500, LERP_TOLERANCE = 10;
    public static double DEVIATION_COEFFICIENT = 200;
    public static double KP = 0.03, KD = 0.002;

    private final GoBildaPinpointDriver odo;

    private final PDController headingController;
    private final LerpController lerpController;

    private double integratedHeading = 0, previousHeading = 0;

    public HeadingController(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.resetPosAndIMU();

        headingController = new PDController(KP, KD);
        lerpController = new LerpController(LERP_RATE, LERP_TOLERANCE);
    }

    public void resetHeading() {
        integratedHeading = 0;
        previousHeading = 0;
        headingController.reset();
        lerpController.reset();
    }
    public void resetHeading(double heading) {
        integratedHeading = heading;
        previousHeading = heading;
        headingController.reset();
        lerpController.reset();
    }

    public void deviateTargetHeading(double deviation, double dt) {
        double targetHeading = lerpController.getEndPosition() - DEVIATION_COEFFICIENT * deviation * dt;
        lerpController.setEndPosition(targetHeading);
    }

    public void snapTargetHeading(Direction direction) {
        double targetHeading = lerpController.getEndPosition();
        if (targetHeading % 90 == 0) {
            int deviation = direction == Direction.ANTICLOCKWISE ? -90 : 90;
            lerpController.setEndPosition(targetHeading + deviation);
        } else if (direction == Direction.ANTICLOCKWISE) {
            lerpController.setEndPosition(Math.floor(targetHeading / 90) * 90);
        } else if (direction == Direction.CLOCKWISE) {
            lerpController.setEndPosition(Math.ceil(targetHeading / 90) * 90);
        }
    }

    public void setTargetHeading(double targetHeading) {
         
        double diff = AngleUnit.normalizeDegrees(targetHeading - integratedHeading);
        lerpController.setEndPosition(integratedHeading + diff);
    }

    public double calculate() {
        headingController.setPID(KP, 0, KD);
        lerpController.setRateAndTolerance(LERP_RATE, LERP_TOLERANCE);

        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        double currentHeading = Math.toDegrees(odo.getHeading(AngleUnit.RADIANS));
        double deltaHeading = AngleUnit.normalizeDegrees(currentHeading - previousHeading);

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return headingController.calculate(integratedHeading, lerpController.calculate());
    }

    public enum Direction {ANTICLOCKWISE, CLOCKWISE}
}
