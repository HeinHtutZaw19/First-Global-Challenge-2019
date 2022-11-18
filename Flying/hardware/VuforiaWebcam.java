package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.Operation;
import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.util.Direction;
import org.flyingdutchman.ftc.robotcore.util.Vector2f;
import org.flyingdutchman.ftc.robotcore.util.Vector3f;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;
import java.util.ArrayList;


public class VuforiaWebcam implements Operation {
    private VuforiaFG2019 vuforia;
    private WebcamName webcam;
    
    public VuforiaWebcam(String name) {
        webcam = OpModeEx.hardware.get(WebcamName.class, name);
        vuforia = new VuforiaFG2019();
        OpModeEx.addOperation(this, 0);
    }
    
    public void setCameraPosition(Vector3f pos, float angle) {
        if(webcam != null) {
            try {
                vuforia.initialize(
                    webcam,
                    false, //USE_EXTENDED_TRACKING
                    false, //ENABLE_CAMERA_DISPLAY
                    pos.y,
                    -pos.x,
                    pos.z,
                    angle - 90,
                    180,
                    0
                );
            }
            catch(Exception e) {
                webcam = null;
            }
        }
    }
    
    /*
    public static final String DUBAI_2019_TARGET = "Dubai_2019";
    public static final String FIRST_GLOBAL_LOGO_TARGET = "FIRST_Global_Logo";
    public static final String REV_ROBOTICS_LOGO_TARGET = "REV_Robotics_Logo";
    public static final String WORLD_MAP_TARGET = "World_Map";
    */
    
    ArrayList<Target> targets = new ArrayList<>();
    
    public void addTargetImage(String name, Vector3f pos, float orientation) {
        targets.add(new Target(name, pos, orientation));
    }
    
    private boolean visible = false;
    private Vector2f position = new Vector2f(0, 0);
    private float heading;
    
    public Vector2f getPosition() { return position; }
    
    public float getHeading() { return heading; }
    
    public boolean isImageDetected() { return visible; }
    
    @Override
    public void start() {
        if(webcam != null) {
            try {
                vuforia.activate();
            }
            catch(Exception e) {
                webcam = null;
                start();
            }
        }
        else {
            visible = false;
            heading = Float.NaN;
            position = null;
        }
    }
    
    @Override
    public void stop() {
        targets.clear();
    }
    
    @Override
    public void loop(double dt) {
        if(webcam == null)
            return;
        
        visible = false;
        position = null;
        heading = Float.NaN;
        
        for(int i=0; i<targets.size(); i++) {
            Target target = targets.get(i);
            VuforiaBase.TrackingResults results = vuforia.track(target.name);
            visible = results.isVisible;
            
            if(results.isVisible) {
                Vector2f rR_I
                = new Vector2f(results.x, results.y).rotate(target.heading);
                rR_I = rR_I.divide(10); // Convert mm to cm
                position = target.pos.add(rR_I); // rR = rI + rR/I
                
                float a = results.zAngle;
                if(a < -90 || a > 90)
                    a += 180;
                heading = Direction.convert(-target.heading + a - 90);
            }
        }
    }
    
    @Override
    public boolean isBusy() { return false; }
}

class Target {
    public String name;
    public Vector2f pos;
    public float heading;
    
    public Target(String str, Vector3f p, float t) {
        name = str;
        pos = new Vector2f(p.x, p.y);
        heading = t;
    }
}
