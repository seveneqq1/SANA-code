package org.firstinspires.ftc.teamcode.bobot.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;


@Configurable
public class ShooterControllerdavid {
    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private VoltageSensor voltageSensor;

    public static double[] calibDistances = {40   ,70   ,120  ,140};
    public static double[] calibRPMs =      {900 ,1200 ,1550 ,1600};
    public static double[] calibServoPos =  {0  ,0.55    ,1    ,1};

    public static boolean useDistanceCompensation = true;

    private double directionPos = 0.7;

 
    public static double kV = 0.0048;
 

    public static double kP = 0.025;
    public static double kI = 0.0;
    public static double kD = 0.000;

    private double targetVelocityRPM = 0;
    private double lastError = 0;
    private double integral = 0;
    private double lastVelocity = 0;
    private long lastTime = 0;

    public static boolean isGraph = true;


    public static double servoClose = 0.5;
    public static double servoLong = 1;

    public enum ServosPos {
        DIRECTION_DOWN(0.6),
        DIRECTION_UP(0.3);

        private final double position;
        ServosPos(double pos) { this.position = pos; }
        public double getPos() { return position; }
    }

    public void initialize(HardwareMap hardwareMap, String shooterMotorLeftName,
                           String shooterMotorRightName, String servoAngleRightName, double pos) {
        shooterMotorLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, shooterMotorLeftName));
        shooterMotorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, shooterMotorRightName));
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        lastTime = System.nanoTime();
    }

    public boolean checkVelocity(double targetVelocity, double offset) {
        double current = Math.abs(-shooterMotorLeft.getVelocity());
        return Math.abs(current - targetVelocity) <= offset;
    }

    public void setShooterVelocity(double targetRPM) {
        this.targetVelocityRPM = targetRPM;
    }


    public void setParametersForDistance(double distance) {
        if (!useDistanceCompensation) return;

        double rpm = interpolate(calibDistances, calibRPMs, distance);
        double servo = interpolate(calibDistances, calibServoPos, distance);

        setShooterVelocity(rpm);
    }

    private double interpolate(double[] xVals, double[] yVals, double x) {
        if (x <= xVals[0]) return yVals[0];
        if (x >= xVals[xVals.length-1]) return yVals[yVals.length-1];

        for (int i = 0; i < xVals.length-1; i++) {
            if (x >= xVals[i] && x <= xVals[i+1]) {
                double t = (x - xVals[i]) / (xVals[i+1] - xVals[i]);
                return yVals[i] + t * (yVals[i+1] - yVals[i]);
            }
        }
        return yVals[0];
    }


    public void update() {
        if (targetVelocityRPM == 0) {
            shooterMotorLeft.setPower(0);
            shooterMotorRight.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) * 1e-9;
        lastTime = currentTime;

        double currentVelocity = Math.abs(-shooterMotorLeft.getVelocity());

        double acceleration = (currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;

        double feedforwardVoltage = kV * targetVelocityRPM;

        double error = targetVelocityRPM - currentVelocity;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidVoltage = kP * error + kI * integral + kD * derivative;

        double totalVoltage = feedforwardVoltage + pidVoltage;

        double batteryVoltage = voltageSensor.getVoltage();
        double power = totalVoltage / batteryVoltage;
        power = Math.max(-1.0, Math.min(1.0, power));

        shooterMotorLeft.setPower(power);
        shooterMotorRight.setPower(power);
    }

    public boolean isReadyToShoot(double tolerance) {
        if (targetVelocityRPM == 0) return false;
        double currentVelocity = Math.abs(-shooterMotorLeft.getVelocity());
        return Math.abs(currentVelocity - targetVelocityRPM) < tolerance;
    }

    public void sendGraphData(PanelsTelemetry dashboard) {
        double currentVelocity = Math.abs(-shooterMotorLeft.getVelocity());
        double target = targetVelocityRPM;

        TelemetryManager panelsTelemetry = dashboard.getTelemetry();
        panelsTelemetry.addData("target",target);
        panelsTelemetry.addData("current",currentVelocity);

        panelsTelemetry.update();
    }

    public double getCurrentVelocity(){
        return Math.abs(-shooterMotorLeft.getVelocity());
    }

    public void showTelemetry(Telemetry telemetry) {
        double currentVelocity = Math.abs(-shooterMotorLeft.getVelocity());

        telemetry.addLine("=== Shooter Controller (PIDF) ===");
        telemetry.addData("Left Encoder (tick/s)", Math.abs(shooterMotorLeft.getVelocity()));
        telemetry.addData("Right Encoder (tick/s)", Math.abs(shooterMotorRight.getVelocity()));
        telemetry.addData("Target (tick/s)", "%.1f", targetVelocityRPM);
        telemetry.addData("Current (tick/s)", "%.1f", currentVelocity);
        telemetry.addData("Error (tick/s)", "%.1f", targetVelocityRPM - currentVelocity);
        telemetry.addData("Motor Power", "%.3f", shooterMotorLeft.getPower());
        telemetry.addData("Ready to Shoot", isReadyToShoot(50) ? "YES" : "NO");
        telemetry.addData("Battery Voltage", "%.2f", voltageSensor.getVoltage());
    }
}