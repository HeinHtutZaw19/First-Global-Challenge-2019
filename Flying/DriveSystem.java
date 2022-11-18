package org.flyingdutchman.ftc.robotcore;

import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.event.Operation;
import org.flyingdutchman.ftc.robotcore.hardware.IMU;
import org.flyingdutchman.ftc.robotcore.util.Vector2f;


public abstract class DriveSystem implements Operation {
    public abstract void drive(double v, double t);
    public abstract void stopDrive();
    
    public void setMaximumSpeed(double v) { maxSpeed = v; }
    public double getMaximumSpeed() { return maxSpeed; }
    
    protected double maxSpeed = 20;
    protected boolean moving = false;
    
    // Odometry prediction by a drive system
    protected Vector2f velocity = new Vector2f(0, 0);
    protected Vector2f position = new Vector2f(0, 0);
    protected float heading;
    
    protected boolean measurementsEnabled = false;
    
    public void enableMeasurements() {
        measurementsEnabled = true;
    }
    
    public Vector2f getVelocity() { return velocity; }
    
    public Vector2f getPosition() { return position; }
    
    public void setPosition(Vector2f pos) { position = pos; }
    
    public float getHeading() { return heading * (float)(180/Math.PI); }
    
    public void setHeading(float t) { heading = t * (float)(Math.PI/180); }
    
    @Override
    public boolean isBusy() { return false; }
}
