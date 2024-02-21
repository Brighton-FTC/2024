package org.firstinspires.ftc.teamcode.localisationStack;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.kalmanFilter.KalmanFilter;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.opencv.core.Point;

import java.util.HashMap;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.Random;
import java.util.stream.Collectors;

class LocalisationStack implements Localizer {
    public static final double FIELD_SIZE = DistanceUnit.INCH.toMeters(72);

    public static final double ROBOT_LENGTH = 0.5;
    public static final double ROBOT_WIDTH = 0.5;

    private final double headingOffset;

    private final SensorDistanceEx frontSensor, backSensor, leftSensor, rightSensor;

    private final BCGyro gyro;

    private INDArray deltaX;
    private INDArray deltaY;

    private INDArray lastCoords;
    private INDArray z;
    LocalisationStack(HardwareMap hardwareMap, double headingOffset) {
        this.headingOffset = headingOffset < 0 ? headingOffset + 360 : headingOffset;

        lastCoords.add((Number) 1001);
        lastCoords.add((Number) 1001);


        frontSensor = new SensorRevTOFDistance(hardwareMap, "front_sensor");
        backSensor = new SensorRevTOFDistance(hardwareMap, "back_sensor");
        rightSensor = new SensorRevTOFDistance(hardwareMap, "right_sensor");
        leftSensor = new SensorRevTOFDistance(hardwareMap, "left_sensor");

        gyro = new BCGyro(hardwareMap, "gyro");

    }
    @NonNull
    public Pose2d setPoseEstimate() {

        this.z.add((Number)frontSensor);
        this.z.add((Number) leftSensor);
        this.z.add((Number) backSensor);
        this.z.add((Number) rightSensor);
        INDArray xhatTest = Nd4j.create(new float[]{1001, 1001}, new int[]{2, 1});
        if (this.lastCoords == xhatTest) {
            this.deltaX = Nd4j.create(new float[]{0, 0, 0, 0}, new int[]{4, 1});
            this.deltaY = Nd4j.create(new float[]{0,0,0,0}, new int[]{4, 1});
        }
        INDArray p0 = Nd4j.eye(5);
        float t = 1;
        this.z.add(gyro.getAbsoluteHeading());
        float discretizationStep = 0.1F;
        INDArray x = Nd4j.create(new float[]{0, 0, 0}, new int[] {4, 1});
        float h = 0.1F;
        INDArray a = Nd4j.create(new float[] {1,h, (float) (0.5*(Math.pow(h, 2))),0, 1, h,0,0,1}, new int[] {3, 3});
        INDArray b = Nd4j.create(new float[] {0, 0, 0}, new int[] {3, 1});
        INDArray c = Nd4j.create(new float[] {1,0, 0}, new int[]{3, 1});
        INDArray q = Nd4j.create(new float[] {0, 0, 0, 0, 0, 0, 0, 0, 0}, new int[] {3, 3});
        INDArray r = Nd4j.create(new float[] {0}, new int[] {1, 1});


        KalmanFilter kalmanFilter = new KalmanFilter(x,  p0,  a,  b,  c,  q,  r);
        kalmanFilter.propagateDynamics(Nd4j.create(new float[]{0}, new int[] {0}));
        kalmanFilter.computeAposterioriEstimate(z);
        INDArray estimatesAposteriori = kalmanFilter.getEstimates_aposteriori();
        return new Pose2d(estimatesAposteriori.getDouble(0, 0)/Math.cos(estimatesAposteriori.getDouble(0, 4)), estimatesAposteriori.getDouble(0, 1), estimatesAposteriori.getDouble(0, 4));
    }



