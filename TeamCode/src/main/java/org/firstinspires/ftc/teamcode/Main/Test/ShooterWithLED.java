package org.firstinspires.ftc.teamcode.Main.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ShooterWithLED extends LinearOpMode {

    private DcMotorEx shooterLeft;
    private ConfigLED bench = new ConfigLED();

    @Override
    public void runOpMode() {

        // --- Initialize LEDs ---
        bench.init(hardwareMap);

        // --- Initialize shooter motor ---
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLeft.setVelocityPIDFCoefficients(100, 0, 20, 12);

        double targetRPM = 6000;
        double gearRatio = 1.0;  // change if you have gearing
        double ticksPerRev = 28 * gearRatio;
        double targetTPS = targetRPM * ticksPerRev / 60.0;

        waitForStart();

        while (opModeIsActive()) {

            // --- Spin shooter when A is pressed ---
            if (gamepad1.a) {
                shooterLeft.setVelocity(targetTPS);
            } else {
                shooterLeft.setPower(0);
            }

            // --- Read current velocity ---
            double currentTPS = shooterLeft.getVelocity();
            double currentRPM = currentTPS / ticksPerRev * 60.0;

            // --- LED logic ---
            if (currentRPM >= targetRPM - 100) { // green if near 6000 RPM
                bench.setGreenLed(true);
                bench.setRedLed(false);
            } else { // red if too low
                bench.setGreenLed(false);
                bench.setRedLed(true);
            }

            // --- Optional telemetry ---
            telemetry.addData("RPM", currentRPM);
            telemetry.update();
        }
    }
}
