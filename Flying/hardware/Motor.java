package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.Operation;
import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.util.Transition;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;


/*
 * Contains current transition function.
 * Prevents power flactuation problems.
 */
public class Motor implements Operation {
    protected DcMotor motor;
    
    public Motor(String name) {
        motor = OpModeEx.hardware.dcMotor.get(name);
        OpModeEx.addOperation(this, 0);
    }
    
    protected Transition powerTransition = new Transition(5);
    
    protected double power;
    
    public void setPower(double p) {
        powerTransition.setTarget(p);
    }
    
    public double getPower() { return power; }
    
    public void setPowerChangeRate(double rate) {
        powerTransition = new Transition(rate);
    }
    
    public void setDirection(Direction d) {
        motor.setDirection(d);
    }
    
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }
    
    @Override
    public void start() {}
    
    @Override
    public void stop() {
        setPower(0);
        power = 0;
        motor.setPower(0);
    }
    
    @Override
    public void loop(double dt) {
        power = powerTransition.getCurrent(dt);
        motor.setPower(power);
    }
    
    @Override
    public boolean isBusy() {
        return powerTransition.isReached();
    }
}
