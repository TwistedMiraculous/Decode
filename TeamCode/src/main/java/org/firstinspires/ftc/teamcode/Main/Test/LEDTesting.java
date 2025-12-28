package org.firstinspires.ftc.teamcode.Main.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class LEDTesting extends LinearOpMode {
    ConfigLED bench = new ConfigLED();
    @Override
    public void runOpMode() {
        // Initialize LEDs
        bench.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                bench.setRedLed(true);
                bench.setGreenLed(false);
            }

            else if (gamepad1.b) {
                bench.setRedLed(false);
                bench.setGreenLed(true);
            }

        }
    }
}
