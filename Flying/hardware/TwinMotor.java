package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.util.PID;
import org.flyingdutchman.ftc.robotcore.util.PIDF;
import org.flyingdutchman.ftc.robotcore.util.Transition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;


/*
 * Two motors with synchronized motion.
 * 2 PID blocks for positioning of each motor and 1 PID block for
 * synchronization.
 */
public class TwinMotor extends EncoderMotor {
    protected DcMotor motor2;
    protected double power2;
    protected double velocityRaw2;
    protected Transition powerTransition2 = new Transition(5);
    protected PIDF position2PIDF = new PIDF(0.00547, 0, 0.000298, 0);
    protected PID adjustingPID = new PID(0, 0, 0);
    
    public TwinMotor(String name1, String name2){
        super(name1);
        motor2 = OpModeEx.hardware.dcMotor.get(name2);
        if(motor.getMotorType().getTicksPerRev()
           != motor2.getMotorType().getTicksPerRev())
        {
            throw new RuntimeException("Motors of same type required");
        }
        setPowerChangeRate(40);
    }
    
    @Override
    public void setPositionPID(double p, double i, double d) {
        positionPIDF.setCoefficients(p, i, d, 0);
        position2PIDF.setCoefficients(p, i, d, 0);
    }
    
    @Override
    public void setPositionPIDF(double p, double i, double d, double f) {
        positionPIDF.setCoefficients(p, i, d, f);
        position2PIDF.setCoefficients(p, i, d, f);
    }
    
    public void setAdjustingPID(double p, double i, double d) {
        adjustingPID.setCoefficients(p, i, d);
    }
    
    public void setDirection(Direction d1, Direction d2) {
        motor.setDirection(d1);
        motor2.setDirection(d2);
    }
    
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
        motor2.setZeroPowerBehavior(behavior);
    }
    
    public double getPosition() {
        return (getPosition1() + getPosition2()) / 2;
    }
    
    public double getPosition1() {
        return 360 * ((motor.getCurrentPosition()) / ticksPerRev);
    }
    
    public double getPosition2() {
        return 360 * ((motor2.getCurrentPosition()) / ticksPerRev);
    }

    public double getVelocity() {
        return velocity * (Math.PI / 180);
    }
    
    public double getVelocityRaw1() {
        return velocityRaw * (Math.PI / 180);
    }
    
    public double getVelocityRaw2() {
        return velocityRaw2 * (Math.PI / 180);
    }
    
    @Override
    public void setPowerChangeRate(double rate) {
        powerTransition  = new Transition(rate);
        powerTransition2 = new Transition(rate);
    }
    
    @Override
    public void setPower(double p) {
        pidMode = PIDMode.NO_PID;
        powerTransition.setTarget(p);
        powerTransition2.setTarget(p);
    }
    
    @Override
    public double getPower() { return (power + power2) / 2; }
    
    public double getPower1() { return power; }
    
    public double getPower2() { return power2; }
    
    
    protected double lastPosition2;
    
    @Override
    public void start() {
        lastPosition  = getPosition1();
        lastPosition2 = getPosition2();
    }
    
    @Override
    public void loop(double dt) {
        double currentPosition1 = getPosition1();
        double currentPosition2 = getPosition2();

        // PID control for position error between twin motors
        double diff = currentPosition2 - currentPosition1;
        double out_diff = adjustingPID.getControllerOutput(0, diff, dt);
        out_diff = Math.max(-1, Math.min(out_diff, 1));
        
        velocityRaw  = (currentPosition1 - lastPosition) / dt;
        velocityRaw2 = (currentPosition2 - lastPosition2) / dt;
        velocity = filter.getSmoothened((velocityRaw + velocityRaw2) / 2);
        
        if(pidMode == pidMode.POSITION_PID) {
            double out_pos1, out_pos2;
            
            out_pos1 = positionPIDF.getPIDControllerOutput(
                           targetPosition, currentPosition1, dt
                       );
            out_pos1 = Math.max(-1, Math.min(out_pos1, 1));
            
            out_pos2 = positionPIDF.getPIDControllerOutput(
                           targetPosition, currentPosition2, dt
                       );
            out_pos2 = Math.max(-1, Math.min(out_pos2, 1));
            
            double ff;
            if(limitMode == LimitMode.HAS_LIMIT) {
                double d = upperLimit - lowerLimit;
                double posPerc = (targetPosition - lowerLimit) / d;
                if(posPerc < 0.2)
                    ff = positionPIDF.getKf() * d * posPerc;
                else
                    ff = positionPIDF.getKf() * d * 0.2;
            }
            else
                ff = 0;
            out_pos1 += ff;
            out_pos2 += ff;
            
            double out1 = (0.9f*out_pos1) + (0.3f*out_diff);
            double out2 = (0.9f*out_pos2) - (0.3f*out_diff);
            
            powerTransition.setTarget(out1);
            powerTransition2.setTarget(out2);
        }
        
        else if(pidMode == pidMode.VELOCITY_PID) {
            double out_v1, out_v2;
            
            out_v1 = velocityPIDF.getControllerOutput(
                           targetVelocity, velocityRaw, dt
                       );
            out_v1 = Math.max(-1, Math.min(out_v1, 1));
            
            out_v2 = positionPIDF.getControllerOutput(
                           targetVelocity, velocityRaw2, dt
                       );
            out_v2 = Math.max(-1, Math.min(out_v2, 1));
            
            double out1 = (0.9f*out_v1) + (0.3f*out_diff);
            double out2 = (0.9f*out_v2) - (0.3f*out_diff);
            
            powerTransition.setTarget(out1);
            powerTransition2.setTarget(out2);
        }
        
        else {
            double out1 = powerTransition.getTarget() + 0.3f*out_diff;
            double out2 = powerTransition.getTarget() - 0.3f*out_diff;
            powerTransition.setTarget(out1);
            powerTransition2.setTarget(out2);
        }
        
        // Limit constrain function
        if(limitMode == LimitMode.HAS_LIMIT) {
            if((currentPosition1<lowerLimit + limitClearance &&
                powerTransition.getTarget() < 0))
            {
                double perc = (currentPosition1 - lowerLimit) / limitClearance;
                double pow = Math.max(-perc,
                             Math.min(powerTransition.getTarget(), perc));
                powerTransition.setTarget(pow);
            }
            else if((currentPosition1>upperLimit - limitClearance &&
                     powerTransition.getTarget() > 0))
            {
                double perc = (upperLimit - currentPosition1) / limitClearance;
                double pow = Math.max(-perc,
                             Math.min(powerTransition.getTarget(), perc));
                powerTransition.setTarget(pow);
            }
            
            if((currentPosition2<lowerLimit + limitClearance &&
                powerTransition2.getTarget() < 0))
            {
                double perc = (currentPosition2 - lowerLimit) / limitClearance;
                double pow = Math.max(-perc,
                             Math.min(powerTransition2.getTarget(), perc));
                powerTransition2.setTarget(pow);
            }
            else if((currentPosition2>upperLimit - limitClearance &&
                     powerTransition2.getTarget() > 0))
            {
                double perc = (upperLimit - currentPosition2) / limitClearance;
                double pow = Math.max(-perc,
                             Math.min(powerTransition2.getTarget(), perc));
                powerTransition2.setTarget(pow);
            }
        }
        
        power  = powerTransition.getCurrent(dt);
        power2 = powerTransition2.getCurrent(dt);
        motor.setPower(power);
        motor2.setPower(power2);
    }
}
