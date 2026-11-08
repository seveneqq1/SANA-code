package org.firstinspires.ftc.teamcode.bobot.controller;

public class ShotTable {
    private final double[] distances, speeds;

    public ShotTable(double[] distances, double[] speeds) {
        if (distances.length < 2 || distances.length != speeds.length) {
            throw new IllegalArgumentException("Provide at least two distance/TPS pairs");
        }
        this.distances = distances.clone();
        this.speeds = speeds.clone();

    }

    public double getTPS(double distance) {
        if (distance <= distances[0]) return speeds[0];
        for (int i = 1; i < distances.length; i++) {
            if (distance <= distances[i]) {
                double fraction = (distance - distances[i - 1]) / (distances[i] - distances[i - 1]);
                return speeds[i - 1] + fraction * (speeds[i] - speeds[i - 1]);
            }
        }
        return speeds[speeds.length - 1];
    }
}
