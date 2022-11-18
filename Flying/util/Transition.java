package org.flyingdutchman.ftc.robotcore.util;

public class Transition {
    private double rate;
    private double target, current;
    
    public Transition(double rate) {
        this.rate = rate;
    }
    
    public void setTarget(double tar) {
        target = tar;
    }
    
    public double getTarget() { return target; }
    
    public double getCurrent(double dt) {
        double err;
        err = target - current;
        if(Math.abs(rate*dt) > Math.abs(err)) {
            current = target;
            return current;
        }

        current += Math.signum(err) * rate * dt;
        return current;
    }
    
    public boolean isReached() {
        return (current!=target) ? true:false;
    }
}
