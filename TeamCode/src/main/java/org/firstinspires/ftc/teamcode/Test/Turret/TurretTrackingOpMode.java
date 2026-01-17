package org.firstinspires.ftc.teamcode.Test.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Test.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Test.Subsystems.Turret;

@TeleOp
public class TurretTrackingOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Vision vision = new Vision(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        telemetry.addData("Status", "Ready. Point turret forward before Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double tagX = vision.getTagX();

            // 1. Core Logic
            turret.trackTarget(tagX);
            vision.updateDashboardExposure();

            // 2. DASHBOARD GRAPHING LOGIC
            TelemetryPacket packet = new TelemetryPacket();

            // Setpoint is 320 (Middle of the 640px camera)
            packet.put("1. Target X", 320);

            // Current position (if -1, we plot it as 320 so the graph doesn't "drop" during search)
            double graphX = (tagX == -1) ? 320 : tagX;
            packet.put("2. Actual X", graphX);

            // Add a "Search Status" line (0 = tracking, 100 = searching)
            packet.put("3. Search Mode", (tagX == -1) ? 100 : 0);

            dashboard.sendTelemetryPacket(packet);

            // 3. STANDARD TELEMETRY (For Driver Station)
            telemetry.addLine("--- VISION DATA ---");
            telemetry.addData("Tag X Position", tagX);
            telemetry.addData("Targeting", (tagX != -1) ? "LOCKED" : "LOST - Searching...");
            telemetry.addData("Encoder Ticks", turret.getPosition());
            telemetry.update();
        }

        vision.close();
        turret.stop();
    }
}