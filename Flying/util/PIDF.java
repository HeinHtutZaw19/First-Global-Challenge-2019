package org.flyingdutchman.ftc.robotcore.util;

public class PIDF extends PID {
    private double Kf;
    
    public PIDF(double p, double i, double d, double f) {
        super(p, i, d);
        Kf = f;
    }
    
    @Override
    public void setCoefficients(double p, double i, double d) {}

    public void setCoefficients(double p, double i, double d, double f) {
        super.setCoefficients(p, i, d);
        Kf = f;
    }
    
    @Override
    public double getControllerOutput(double target, double current, double dt) {
        double out =  super.getControllerOutput(target, current, dt);
        return out + Kf*target;
    }
    
    public double getPIDControllerOutput(double target, double current,
                                         double dt)
    {
        return super.getControllerOutput(target, current, dt);
    }
    
    public double getKf() { return Kf; }
}
