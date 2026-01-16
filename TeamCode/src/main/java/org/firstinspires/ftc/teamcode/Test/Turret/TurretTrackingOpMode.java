package org.firstinspires.ftc.teamcode.Test.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Test.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Test.Subsystems.Turret;

@TeleOp
public class TurretTrackingOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize both subsystems
        Vision vision = new Vision(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        telemetry.addData("Status", "Ready. Check Dashboard for Camera Feed.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Ask the eyes where the tag is
            double tagX = vision.getTagX();

            // 2. Tell the turret to move toward that X position
            turret.trackTarget(tagX);


            telemetry.addData("Tag X Position", tagX);
            if (tagX != -1) {
                telemetry.addData("Targeting", "LOCKED");
            } else {
                telemetry.addData("Targeting", "LOST - Searching...");
            }
            telemetry.update();
        }


        vision.close();
        turret.stop();
    }
}