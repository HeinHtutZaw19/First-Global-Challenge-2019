package org.flyingdutchman.ftc.robotcore.event;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.ArrayList;


/*
 * Extended OpMode. All other classes depends on it.
 * Handles the control loops PIDs, timers, etc.
 * 
 * Whenever the subclasses override init, start, loop, they should
 * call the superclass functions to work well with the library classes.
 */
public abstract class OpModeEx extends OpMode {
    public static Gamepad gamepad;
    public static HardwareMap hardware;
    
    public static String home = "/storage/emulated/0/FIRST/flyingdutchman/";
    
    @Override
    public void init() {
        gamepad = gamepad1;
        hardware = hardwareMap;
        Monitor.init(telemetry);
    }
    
    @Override
    public void init_loop() {
        GamepadButton.loop();
    }
    
    @Override
    public void start() {
        stopRequest = false;
        
        for(int i=0; i<ops.size(); i++)
            ops.get(i).start();
        
        GamepadButton.enableCombo();
        t1 = time - 0.001;
    }
    
    private static double t1 = 0;
    public static double now() {return t1;}
    
    private static ArrayList<Operation> ops = new ArrayList<Operation>();
    
    public static boolean stopRequest = false;
    
    @Override
    public void loop() { _loop(); }
    
    public void _loop() {
        double t2 = time;
        double dt = t2 - t1;
        t1 = t2;
        
        for(int i=0; i<ops.size(); i++) {
            ops.get(i).loop(dt);
        }
        
        Monitor.addData("dt", String.format("%dms", (int)(dt*1000)));
        
        GamepadButton.loop();
        telemetry.update();
    }
    
    /*
     * 0 - for general purpose (default)
     * 1 - for slow and not performance critical jobs
     * 2 - for performance critical jobs
     */
    public static void addOperation(Operation op, int i) {
        switch(i) {
          case 0:
            ops.add(op);
        }
    }
    
    @Override
    public void stop() {
        /*
        stopRequest = true;
        
        // Final loops
        boolean terminated = false;
        while(terminated = false) {
            terminated = true;
            double t2 = time;
            double dt = t2 - t1;
            t1 = t2;
            for(int i=0; i<ops.size(); i++) {
                if(ops.get(i).isBusy()) {
                    terminated = false;
                    ops.get(i).loop(dt);
                }
            }
        }
        */
        
        // Stop functions
        for(int i=0; i<ops.size(); i++)
            ops.get(i).stop();
        ops.clear();
        
        Monitor.reset();
    }
    
    public void delay(double t) {
        ElapsedTime timer = new ElapsedTime();
        double st_time = timer.seconds();
        do {
            _loop();
        } while(timer.seconds() - st_time < t);
    }
}
