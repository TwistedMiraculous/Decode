package org.firstinspires.ftc.teamcode.Main.Test;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class ConfigLED {

    private LED redLed;
    private LED greenLed;

    public void init(HardwareMap hwMap) {
        redLed = hwMap.get(LED.class,"ledred");
        greenLed = hwMap.get(LED.class,"ledgreen");
    }

    public void setRedLed(boolean isOn){
        if (isOn){
            redLed.on();
        }
        else {
            redLed.off();
        }
    }

    public void setGreenLed(boolean isOn){
        if (isOn){
            greenLed.on();
        }
        else {
            greenLed.off();
        }
    }
}
