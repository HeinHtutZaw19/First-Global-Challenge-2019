package org.flyingdutchman.ftc.robotcore.event;

import org.flyingdutchman.ftc.robotcore.util.Pair;

import java.util.LinkedList;
import java.util.Queue;


/*
 * Performs time delay functions without pausing the program.
 * Add steps into a queue and are done step by step.
 */
public abstract class TimeStep implements Operation {
    private Queue<Pair<Double, Double>> steps = new LinkedList<>();
    
    public TimeStep() {
        OpModeEx.addOperation(this, 0);
    }
    
    public void next(double val, double t) {
        steps.add(new Pair<>(val, t));
        if(busy == false) {
            t1 = OpModeEx.now();
            busy = true;
        }
    }
    
    public void clear() {
        steps.clear();
    }
    
    protected abstract void func(double val);
    
    private double t1 = 0;
    private boolean busy = false;
    
    @Override
    public void start() {}
    
    @Override
    public void stop() { steps.clear(); }
    
    @Override
    public void loop(double dt) {
        if(busy) {
            Pair<Double, Double> pair = steps.peek();
            if(pair == null)
                busy = false;
            else {
                double t2 = OpModeEx.now();
                double delay = pair.getRight();
                if(t2 - t1 >= delay) {
                    double val = steps.remove().getLeft();
                    func(val);
                    t1 += delay;
                }
            }
        }
    }
    
    @Override
    public boolean isBusy() { return busy; }
}
