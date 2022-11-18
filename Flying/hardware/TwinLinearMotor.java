package org.flyingdutchman.ftc.robotcore.hardware;

/*
 * Linear actuator made from encoder motor with Rack gear or chain mechanism.
 * Twin motor version.
 */
public class TwinLinearMotor extends TwinMotor {
    private double lengthPerRev;
    
    public TwinLinearMotor(String name1,String name2){
        super(name1,name2);
        limitClearance = 2;
    }
   
    public void setLengthPerRev(double l) {
        lengthPerRev = l;
    }
    
    @Override
    public double getPosition1() {
        return (motor.getCurrentPosition() / ticksPerRev) * lengthPerRev;
    }
    
    @Override
    public double getPosition2() {
        return (motor2.getCurrentPosition() / ticksPerRev) * lengthPerRev;
    }
    
    @Override
    public void setVelocity(double v) {
        pidMode = PIDMode.VELOCITY_PID;
        targetVelocity = v;
    }
}
