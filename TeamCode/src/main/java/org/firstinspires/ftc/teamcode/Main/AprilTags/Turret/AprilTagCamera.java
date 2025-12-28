package org.firstinspires.ftc.teamcode.Main.AprilTags.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagCamera {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTagCamera(HardwareMap hardwareMap, boolean useWebcam) {
        initAprilTag(hardwareMap, useWebcam);
    }

    private void initAprilTag(HardwareMap hardwareMap, boolean useWebcam) {

        aprilTag = new AprilTagProcessor.Builder()
                // Optional tuning
                //.setDrawTagOutline(true)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (useWebcam) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Optional
        //builder.setCameraResolution(new Size(640, 480));
        //builder.enableLiveView(true);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    /** Get all current detections */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /** Convenience: get best (first) detection */
    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        return detections.isEmpty() ? null : detections.get(0);
    }

    /** Stop camera streaming */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /** Resume camera streaming */
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    /** Shut everything down */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
