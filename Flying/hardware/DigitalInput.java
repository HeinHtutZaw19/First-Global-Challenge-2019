package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * For touch sensor, magnetic limit switch or any other digital input devices.
 */
public class DigitalInput {
    private TouchSensor input;
    
    public DigitalInput(String name) {
        input = OpModeEx.hardware.touchSensor.get(name);
    }
    
    public boolean getState() {
        return input.isPressed();
    }
}
