package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.OpModeEx;

/*
 * Not much difference from FTC's AnalogInput class.
 * Custom constructor taking a single String.
 */
public class AnalogInput {
    private com.qualcomm.robotcore.hardware.AnalogInput input;
    
    public AnalogInput(String name) {
        input = OpModeEx.hardware.analogInput.get(name);
    }
    
    public double getInput() {
        return input.getVoltage() / input.getMaxVoltage();
    }
    
    public double getVoltage() {
        return input.getVoltage();
    }
}
