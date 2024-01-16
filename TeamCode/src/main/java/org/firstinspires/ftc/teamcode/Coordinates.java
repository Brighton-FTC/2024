package org.firstinspires.ftc.teamcode;

public class Coordinates {
    public double X;
    public double Y;
    public double AnomalyScore;

    Coordinates(double x, double y) {
        X = x;
        Y = y;
    }

    public void setAnomalyScore(double anomalyScore) {
        AnomalyScore = anomalyScore;
    }
}
