package org.flyingdutchman.ftc.robotcore;

import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.event.Monitor;
import org.flyingdutchman.ftc.robotcore.hardware.EncoderMotor;
import org.flyingdutchman.ftc.robotcore.hardware.IMU;
import org.flyingdutchman.ftc.robotcore.util.Vector2f;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;


public class TwoWheelDrive extends DriveSystem {
    private EncoderMotor left;
    private EncoderMotor right;
    
    public TwoWheelDrive(String lname, String rname) {
        left = new EncoderMotor(lname);
        right = new EncoderMotor(rname);
        left.setDirection(Direction.REVERSE);
        right.setDirection(Direction.FORWARD);
        OpModeEx.addOperation(this, 0);
        left.move(0);
        right.move(0);
    }
    
    private float wheelRadius;
    private float wheelAxisLength;
    private Vector2f wheelAxisCenter;
    
    public void setPhysicalParameters(float radius,
                                      float axisLength,
                                      float axisDistance)
    {
        wheelRadius = radius;
        wheelAxisLength = axisLength;
        wheelAxisCenter = new Vector2f(axisDistance, 0);
    }
    
    
    public void setPositionPID(double p, double i, double d) {
        left.setPositionPID(p, i, d);
        right.setPositionPID(p, i, d);
    }
    
    public void setVelocityPIDF(double p, double i, double d, double f) {
        left.setVelocityPIDF(p, i, d, f);
        right.setVelocityPIDF(p, i, d, f);
    }
    
    
    // Continuous function of 2 variables
    @Override
    public void drive(double v, double t) {
        if(Math.abs(v) < 0.1*maxSpeed) {
            stopDrive();
            return;
        }
        
        v = Math.max(-maxSpeed, Math.min(v, maxSpeed));
        
        t *= Math.PI/180; // Converts degree to radian
        if(t>=0 && t < Math.PI/2) {
            left.setVelocity(v * Math.cos(2*t));
            right.setVelocity(v);
        }
        else if(t >= Math.PI/2 && t <= Math.PI) {
            left.setVelocity(-v);
            right.setVelocity(-v * Math.cos(2*t));
        }
        else if(t >= -Math.PI/2 && t<0) {
            left.setVelocity(v);
            right.setVelocity(v * Math.cos(2*t));
        }
        else if(t >= -Math.PI && t < -Math.PI/2) {
            left.setVelocity(-v * Math.cos(2*t));
            right.setVelocity(-v);
        }
        
        moving = true;
    }
    
    private double time1 = 0;
    
    @Override
    public void stopDrive() {
        if(moving == true) {
            left.move(0);
            right.move(0);
            moving = false;
            time1 = OpModeEx.now();
        }
    }
    
    
    // Odometry prediction
    @Override
    public void loop(double dt) {
        if(moving == false) {
            if((OpModeEx.now() - time1 > 0.25) && (time1 > 0)) {
                left.move(0);
                right.move(0);
                time1 = -1;
            }
        }
        
        if(measurementsEnabled == false)
            return;
        
        float v1, v2, v, c, w;
        v1 = wheelRadius * (float)left.getVelocityRaw();
        v2 = wheelRadius * (float)right.getVelocityRaw();
        v = (v1+v2)/2;
        c = wheelAxisLength;
        
        w = (v2-v1)/c;
        float vx = v * (float)Math.cos(heading + w/2);
        float vy = v * (float)Math.sin(heading + w/2);
        Vector2f vA = new Vector2f(vx, vy);
        //Vector2f vO_A = wheelAxisCenter.cross(w); // vO/A = w x rO/A = rA/O x w
        velocity = vA; //.add(vO_A);
        heading += w * (float)dt;
        position = position.add(velocity.multiply((float)dt)); // s = s0 + v*dt
    }
    
    @Override
    public void start() {}
    
    @Override
    public void stop() {}
}
