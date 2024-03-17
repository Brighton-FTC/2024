package org.firstinspires.ftc.teamcode.opModes.autonomous;

import static org.firstinspires.ftc.teamcode.opModes.autonomous.AutonomousGeneric.CAMERA_RESOLUTION;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "April Tag Localization Tester", group = "test")
public class AprilTagLocalizationTester extends OpMode {
    private AprilTagLocalizer ataglocal;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .setCameraResolution(CAMERA_RESOLUTION)
                .build();
        ataglocal = new AprilTagLocalizer(aprilTag);

    }

    @Override
    public void loop() {
        ataglocal.update();
        if (ataglocal.currentPose != null){
            telemetry.addData("heading: ", ataglocal.currentPose.getHeading());
            telemetry.addData("Y: ", ataglocal.currentPose.getY());
            telemetry.addData("X", ataglocal.currentPose.getX());
        }
        else {
            telemetry.addLine("No tags detected");
        }
        telemetry.update();
    }

    @Override
    public void stop(){
        visionPortal.close();
    }
}
