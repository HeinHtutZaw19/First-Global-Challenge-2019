package org.flyingdutchman.ftc.robotcore.hardware;

/*
 * Linear actuator made from encoder motor with Rack gear or chain mechanism.
 * Converts angular unit to length unit.
 */
public class LinearMotor extends EncoderMotor {
    private double lengthPerRev;
    
    public LinearMotor(String name) {
        super(name);
        limitClearance = 2;
    }
    
    public void setLengthPerRev(double l) {
        lengthPerRev = l;
    }
    
    @Override
    public double getPosition() {
        return (motor.getCurrentPosition() / ticksPerRev) * lengthPerRev;
    }
    
    @Override
    public void setVelocity(double v) {
        pidMode = PIDMode.VELOCITY_PID;
        targetVelocity = v;
    }
    
    @Override
    public double getVelocity() {
        return (super.getVelocity() / (2*Math.PI)) * lengthPerRev;
    }
}
