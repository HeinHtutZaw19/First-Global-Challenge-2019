package org.flyingdutchman.ftc.robotcore.event;

import java.util.LinkedList;
import java.util.Queue;


/*
 * Performs step functions without interruptions.
 */
public abstract class Step implements Operation {
    private Queue<Double> steps = new LinkedList<>();
    private Operation device;
    
    public Step(Operation device) {
        this.device = device;
        OpModeEx.addOperation(this, 0);
    }
    
    public void next(double val) {
        steps.add(val);
        busy = true;
    }
    
    public void clear() {
        steps.clear();
    }
    
    protected abstract void func(Operation device, double val);
    
    private boolean busy = false;
    
    @Override
    public void start() {}
    
    @Override
    public void stop() { steps.clear(); }
    
    @Override
    public void loop(double dt) {
        if(busy) {
            if(device.isBusy() == false) {
                Double val = steps.remove();
                if(val == null)
                    busy = false;
                else
                    func(device, val);
            }
        }
    }
    
    @Override
    public boolean isBusy() { return busy; }
}
