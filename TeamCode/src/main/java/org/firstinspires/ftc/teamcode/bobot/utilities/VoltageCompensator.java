package org.firstinspires.ftc.teamcode.bobot.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.bobot.config.RobotConfig;

public class VoltageCompensator {
    private final DoubleSupplier readVoltage;
    private double voltage = Double.NaN;
    private long lastReadNanos;

    public VoltageCompensator(HardwareMap map) { this(() -> highestReading(map)); }

    public VoltageCompensator(DoubleSupplier readVoltage) { this.readVoltage = readVoltage; }

    private static double highestReading(HardwareMap map) {
        double best = 0;
        for (VoltageSensor sensor : map.voltageSensor) best = Math.max(best, sensor.getVoltage());
        return best;
    }

    public double getVoltage() {
        long now = System.nanoTime();
        boolean due = Double.isNaN(voltage) || (now - lastReadNanos) / 1e6 >= RobotConfig.VOLTAGE_READ_INTERVAL_MS;
        if (due) {
            lastReadNanos = now;
            double reading = readVoltage.getAsDouble();
            if (Double.isFinite(reading) && reading > RobotConfig.VOLTAGE_MIN_VALID) {
                double alpha = ControlMath.clamp(RobotConfig.VOLTAGE_FILTER_ALPHA, 0, 1);
                voltage = Double.isNaN(voltage) ? reading : voltage + alpha * (reading - voltage);
            }
        }
        return Double.isNaN(voltage) ? RobotConfig.NOMINAL_VOLTAGE : voltage;
    }

    public double scale() {
        if (!RobotConfig.VOLTAGE_COMPENSATION) return 1;
        return ControlMath.clamp(RobotConfig.NOMINAL_VOLTAGE / getVoltage(), 0, RobotConfig.VOLTAGE_MAX_SCALE);
    }

    public double compensate(double power) { return ControlMath.clamp(power * scale(), -1, 1); }
}
