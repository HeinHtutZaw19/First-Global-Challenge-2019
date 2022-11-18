package org.flyingdutchman.ftc.robotcore.util;

public class DirectionPID extends PID {
    public DirectionPID(double p, double i, double d) {
        super(p, i, d);
    }
    
    public double getControllerOutput(double target, double current, double dt) {
        err = Direction.convert((float)(target - current));
        double P = err;
        I = I + err*dt;
        double D = (err-lastError) / dt;
        lastError = err;
        return Kp*P + Ki*I + Kd*D;
    }
    
    public double getControllerOutput(Direction target, Direction current) {
        return getControllerOutput(target, current, 1);
    }
    
    public double getControllerOutput(Direction target,
                                      Direction current,
                                      double dt)
    {
        err = target.getError(current);
        double P = err;
        I = I + err*dt;
        double D = (err-lastError) / dt;
        lastError = err;
        return Kp*P + Ki*I + Kd*D;
    }
}
