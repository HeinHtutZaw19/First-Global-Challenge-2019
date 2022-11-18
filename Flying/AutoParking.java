package org.flyingdutchman.ftc.robotcore;

import org.flyingdutchman.ftc.robotcore.event.Monitor;
import org.flyingdutchman.ftc.robotcore.event.Operation;
import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.hardware.IMU;
import org.flyingdutchman.ftc.robotcore.hardware.VuforiaWebcam;
import org.flyingdutchman.ftc.robotcore.util.Direction;
import org.flyingdutchman.ftc.robotcore.util.DirectionPID;
import org.flyingdutchman.ftc.robotcore.util.PID;
import org.flyingdutchman.ftc.robotcore.util.Vector2f;


/*
 * Automatically drive the vehicle to target coordinate using two PID blocks.
 * Reads the coordinated from vision targets using Vuforia libraries.
 */
public class AutoParking implements Operation {
    private DriveSystem driveSystem;
    private IMU imuSensor;
    private VuforiaWebcam webcam;
    private Vector2f toolPosition = new Vector2f(0, 0);
    private boolean autoEnabled = false;
    
    public AutoParking(DriveSystem drive, IMU imu, VuforiaWebcam vuforiaWebcam) {
        driveSystem = drive;
        imuSensor = imu;
        webcam = vuforiaWebcam;
        driveSystem.enableMeasurements();
        distancePID.setTolerance(6);
        directionPID.setTolerance(2);
        OpModeEx.addOperation(this, 0);
    }
    
    public void setToolPosition(Vector2f pos) { toolPosition = pos; }
    
    public void setAutoEnabled(boolean en) {
        autoEnabled = en;
        retracting = false;
    }
    
    private Vector2f targetPoint;
    private double targetDistance;
    private PID distancePID = new PID(0.3, 0, 0);
    private DirectionPID directionPID = new DirectionPID(1.5f, 0, 0);
    
    public void setTarget(Vector2f point, double distance) {
        targetPoint = point;
        targetDistance = distance;
        retracting = false;
    }
    
    public void setDistancePID(double p, double i, double d) {
        distancePID = new PID(p, i, d);
    }
    
    public void setDirectionPID(double p, double i, double d) {
        directionPID = new DirectionPID(p, i, d);
    }
   
    @Override
    public void start() { coordinate = driveSystem.getPosition(); }
    
    @Override
    public void stop() {
        retracting = false;
    }
    
    
    private boolean retracting = false;
    private float heading;
    private float imuHeadingDiff;
    private Vector2f coordinate = new Vector2f(0,0);
    
    private boolean imageDetected = false;
    
    private float distance;
    private float directionError;
    
    
    public float getHeading() { return heading; }
    
    public Vector2f getPosition() { return coordinate; }
    
    public float getDistance() { return distance; }
    
    public float getDirectionError() { return directionError; }
    
    
    @Override
    public void loop(double dt) {
        // Measurements
        
        // Heading
        boolean imgDetect = webcam.isImageDetected();
        if(imgDetect) {
            Direction hw = new Direction(webcam.getHeading());
            Direction hg = new Direction(
                heading + imuSensor.getAngularVelocity() * (float)dt
            );
            heading = Direction.linearMap(hg, hw, 0.02f);
            heading = Direction.convert(heading);
            // h = (h + dhg) * 0.98 + hw * 0.02
        }
        else {
            float hg = imuSensor.getHeading();
            if(imageDetected == true)
                imuHeadingDiff = heading - hg;
            heading = Direction.convert(hg + imuHeadingDiff);
        }
        imageDetected = imgDetect;
        Monitor.addData("hdiff", imuHeadingDiff);
        
        // Coordinate
        driveSystem.setHeading(heading);
        Vector2f v = driveSystem.getVelocity();
        Vector2f dpe = v.multiply((float)dt);
        if(imageDetected) {
            Vector2f pw = webcam.getPosition();
            coordinate =
                ((coordinate.add(dpe)).multiply(0.96f)).add(pw.multiply(0.04f));
            // p = (p + dpe) * 0.96 + pw * 0.04
        }
        else
            coordinate = coordinate.add(dpe);
        
        Vector2f rT_W = targetPoint.subtract(coordinate.add(toolPosition));
        distance = rT_W.getMagnitude();
        double targetDir = Math.atan2(rT_W.y, rT_W.x) * 180/Math.PI;
        directionError = Direction.convert((float)(targetDir - heading));
        
        // Automatic functions
        if(autoEnabled == false || targetPoint == null)
            return;
        
        // Output
        double spd = -distancePID.getControllerOutput
                         (targetDistance, distance, dt);
        double dir = directionPID.getControllerOutput(targetDir, heading, dt);
        dir = Math.max(-90, Math.min(dir, 90));
        if(spd < 0) {
            spd *= -1;
            dir = Direction.convert(dir + 180);
        }
        
        Monitor.addData("Output", String.format("%.2f, %.2f", spd, dir));
        driveSystem.drive(spd, dir);
        
        if(retracting == false) {
            if(distancePID.isReached() && directionPID.isReached() == false) {
                targetDistance -= 30;
                retracting = true;
            }
        }
        else if(distancePID.isReached()) {
            targetDistance += 30;
            retracting = false;
        }
    }
    
    @Override
    public boolean isBusy() {
        if(OpModeEx.stopRequest)
            return false;
        
        return distancePID.isReached() == false ||
               directionPID.isReached() == false;
    }
}
