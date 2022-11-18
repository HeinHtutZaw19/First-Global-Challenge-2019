package org.flyingdutchman.ftc.robotcore.util;

public class PID {
    protected double Kp, Ki, Kd;
    protected double err = 0;
    protected double lastError = 0;
    protected double I = 0;
    
    public PID(double p, double i, double d) {
        setCoefficients(p, i, d);
    }
    
    public void setCoefficients(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }
    
    private double tolerance;
    
    public void setTolerance(double tol) { tolerance = tol; }
    
    public double getControllerOutput(double target, double current) {
        return getControllerOutput(target, current, 1);
    }
    
    public double getControllerOutput(double target, double current, double dt) {
        err = target - current;
        double P = err;
        I = I + err*dt;
        double D = (err-lastError) / dt;
        lastError = err;
        return Kp*P + Ki*I + Kd*D;
    }
    
    public boolean isReached() { 
        if(Math.abs(err) < tolerance ||
           Math.signum(err) != Math.signum(lastError))
        {
            return true;
        }
        else
            return false;
    }
}
