package org.firstinspires.ftc.teamcode;

import android.system.ErrnoException;
import android.view.contentcapture.DataRemovalRequest;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.util.ArrayUtils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;

import smile.anomaly.IsolationForest;
import smile.math.matrix.*;
import smile.util.Array2D;


/**
 * Read sensors and take a best guess at where you are
 * Move a vector in a known direction by a known distance (eg by spinning a wheels exactly once and updating estimated position to (previous position + vector)
 * Remeasure the sensors
 * Compare each sensor reading to what your predicted positions says it should be
 * Attach weights to sensors according to accuracy, most accurate highest weight
 * Take the weighted average reading of the sensors (sum of weights*sensor readings) / (product of weights * number of sensors)
 * Use these weighted average readings instead of the actual sensor readings 8. Go to step 2.
 **/

class KalmanFilter {
    private DistanceSensor distanceSensorFL;
    private DistanceSensor distanceSensorFR;
    private DistanceSensor distanceSensorBL;
    private DistanceSensor distanceSensorBR;
    private MecanumDrive mecanumDrive;
    private BCGyro gyro;


    KalmanFilter(DistanceSensor distanceSensorFrontLeft, DistanceSensor distanceSensorFrontRight, DistanceSensor distanceSensorBackLeft, DistanceSensor distanceSensorBackRight, MecanumDrive drive, BCGyro bcGyro) {
        distanceSensorFL = distanceSensorFrontLeft;
        distanceSensorFR = distanceSensorFrontRight;
        distanceSensorBL = distanceSensorBackLeft;
        distanceSensorBR = distanceSensorBackRight;
        mecanumDrive = drive;
        gyro = bcGyro;
        

    }

    private IsolationForestReturn anomalyDetectionIF(Coordinates[] data, double threshold) {
        double[][] array = new double[data.length][2];
        for (int i = 0; i < data.length; i++) {
            array[i][0] = data[i].X;
            array[i][1] = data[i].Y;
        }


        IsolationForest isolationForest = IsolationForest.fit(array);
        for (int j = 0; j < data.length; j++) {
            data[j].setAnomalyScore(isolationForest.score(array[j]));
        }

        ArrayList<Coordinates> anomalies = new ArrayList<Coordinates>();
        ArrayList<Coordinates> prunedData = new ArrayList<Coordinates>();
        for (int k= 0; k < data.length; k++) {
            if (data[k].AnomalyScore > threshold) {
                anomalies.add(data[k]);
            } else {
                prunedData.add(data[k]);
            }
        }

        return new IsolationForestReturn(data, prunedData, anomalies);
    }

    private void turnDegree(double degree, double accuracy) {
        double currentZAngle = gyro.getAbsoluteHeading();
        double measureAngle = degree*2+accuracy;
        mecanumDrive.driveRobotCentric(0, 0, 1);
        while (measureAngle - currentZAngle - degree > accuracy) {
            try {
                TimeUnit.MILLISECONDS.wait(1);
            } catch (InterruptedException e) {
                throw new Error(e);
            }
            measureAngle = gyro.getAbsoluteHeading();
        }
        mecanumDrive.driveRobotCentric(0, 0, 0);
    }

    private Coordinates estimateCoordinateDS(int accuracy) {
        double currentZAngle = gyro.getAbsoluteHeading();
        if (currentZAngle >= 180) {
            turnDegree(360 - currentZAngle, 0.001);
        } else {
            turnDegree(360 + currentZAngle, 0.001);
        }
        Coordinates[] coordinates = new Coordinates[accuracy*2];
        for (int i = 0; i < accuracy; i++){
            double[] distances = {distanceSensorFL.getDistance(DistanceUnit.METER), distanceSensorFR.getDistance(DistanceUnit.METER), distanceSensorBL.getDistance(DistanceUnit.METER), distanceSensorBR.getDistance(DistanceUnit.METER)};
            coordinates[i*2] = new Coordinates(distances[0], distances[1]);
            coordinates[i*2 + 1] = new Coordinates(distances[2], distances[3]);
            turnDegree(359, 0.001);
            turnDegree(1, 0.0001);
        }
        IsolationForestReturn anomalyReturn = anomalyDetectionIF(coordinates, 0.1);
        double dataX = 0;
        double dataY = 0;
        for (int i = 0; i < accuracy*2; i++) {
            dataX += anomalyReturn.PrunedData.get(i).X;
            dataY += anomalyReturn.PrunedData.get(i).Y;
        }
        return new Coordinates(dataX/(accuracy*2), dataY/(accuracy*2));
    }

    public double[] getCoordinates() {
        Coordinates coordinatesFromDS = estimateCoordinateDS(5);

    }




}