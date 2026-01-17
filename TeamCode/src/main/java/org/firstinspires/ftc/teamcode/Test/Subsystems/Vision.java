package org.firstinspires.ftc.teamcode.Test.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config; // Added for Dashboard
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Vision {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;


    public static int EXPOSURE_MS = 6;
    public static int GAIN = 250;

    public Vision(HardwareMap hardwareMap) {
        AprilTagLibrary myTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "My100mmTag", 4.7, DistanceUnit.INCH)
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(myTagLibrary)
                .setLensIntrinsics(622.548, 622.548, 319.5, 239.5)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    /**
     * Updates the camera with the current values from the Dashboard.
     * Call this inside your OpMode's while loop.
     */
    public void updateDashboardExposure() {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }

        // Only update if the values are actually different to save processing power
        if (exposureControl.getExposure(TimeUnit.MILLISECONDS) != EXPOSURE_MS) {
            exposureControl.setExposure((long) EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl.getGain() != GAIN) {
            gainControl.setGain(GAIN);
        }
    }

    public double getTagX() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                return detection.center.x;
            }
        }
        return -1;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}