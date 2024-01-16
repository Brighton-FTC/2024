package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;

public class IsolationForestReturn {
    public Coordinates[] Data;
    public ArrayList<Coordinates> PrunedData;
    public ArrayList<Coordinates> Anomalies;

    IsolationForestReturn(Coordinates[] data, ArrayList<Coordinates> prunedData, ArrayList<Coordinates> anomalies) {
        Data = data;
        PrunedData = prunedData;
        Anomalies = anomalies;
    }


}
