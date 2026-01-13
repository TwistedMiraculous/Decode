package org.firstinspires.ftc.teamcode.Test.Subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagCamera {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public AprilTagCamera(HardwareMap hardwareMap, boolean enablePreview) {
        // 1. Create processor with scaled 1080p intrinsics for C920
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1867.644, 1867.644, 958.5, 718.5)
                .build();

        // 2. Build portal with 1080p MJPEG for 30 FPS
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag);

        if (!enablePreview) {
            builder.enableLiveView(false);
        }

        visionPortal = builder.build();
        setManualCameraSettings();
    }

    private void setManualCameraSettings() {
        long startTime = System.currentTimeMillis();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && System.currentTimeMillis() - startTime < 2000) {
            // Wait for camera to open
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            // Lock focus to prevent focal length drift
            FocusControl focusControl = visionPortal.getCameraControl(FocusControl.class);
            if (focusControl != null && focusControl.isModeSupported(FocusControl.Mode.Fixed)) {
                focusControl.setMode(FocusControl.Mode.Fixed);
            }

            // Set low exposure (8ms) to eliminate motion blur
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null && exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(8, TimeUnit.MILLISECONDS);
            }
        }
    }

    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;

        AprilTagDetection best = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                // Returns the closest tag found
                if (best == null || detection.ftcPose.range < best.ftcPose.range) {
                    best = detection;
                }
            }
        }
        return best;
    }

    public VisionPortal getCamera() { return visionPortal; }

    public void close() {
        if (visionPortal != null) { visionPortal.close(); }
    }
}
