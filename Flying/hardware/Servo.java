package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.*;
import com.qualcomm.robotcore.hardware.Servo.Direction;

/*
 * Converts 0 to 1 servo signals to degrees.
 * Servos require configuration with SRS programmer.
 *
 * Uses powerful time step functions.
 */
public class Servo {
    private com.qualcomm.robotcore.hardware.Servo servo;
    
    public Servo(String name) {
        servo = OpModeEx.hardware.servo.get(name);
    }
    
    private double lowerLimit = 0;
    private double upperLimit = 270;
    
    public void setLimit(double lower, double upper) {
        lowerLimit = lower;
        upperLimit = upper;
    }
    
    public void setDirection(Direction d) {
        servo.setDirection(d);
    }
    
    public boolean isBusy() {
        if(step != null)
            return step.isBusy();
        else
            return false;
    }
    
    
    private TimeStep step = null;
    
    private void enableTimeStep() {
        step = new TimeStep() {
            @Override
            protected void func(double val) {
                servo.setPosition(val);
            }
        };
    }
    
    
    public void setPosition(double deg) {
        if(step == null) {
            deg = (deg - lowerLimit) / (upperLimit - lowerLimit);
            servo.setPosition(deg);
        }
        else {
            if(step.isBusy() == false)
                setNextPosition(deg, 0);
            else
                Monitor.log("Servo busy");
        }
    }
    
    public void setNextPosition(double deg, double t) {
        if(step == null)
            enableTimeStep();
        
        if(deg < lowerLimit || deg > upperLimit) {
            Monitor.log("****************");
            Monitor.log(String.format("setPosition(%.2f)", deg));
            Monitor.log("Servo target out of bounds.");
            Monitor.log("****************");
            if(deg < lowerLimit)
                deg = 0;
            else if(deg > upperLimit)
                deg = 1;
        }
        else
            deg = (deg - lowerLimit) / (upperLimit - lowerLimit);
        
        step.next(deg, t);
    }
    
    public void cancel() {
        if(step != null)
            step.clear();
    }
}
