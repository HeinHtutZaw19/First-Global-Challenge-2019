package org.flyingdutchman.ftc.robotcore.event;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;


/*
 * Interprets the telemetry Monitor as a serial monitor.
 * Stores the logging data.
 */
public class Monitor {
    private static Telemetry telemetry;
    
    public static void init(Telemetry telemetry) {
        Monitor.telemetry = telemetry;
        LogData.setTelemetry(telemetry);
    }
    
    public static void reset() {
        consoleOutput.clear();
        logData.clear();
        telemetry.update();
    }
    
    private static ArrayList<LogData> logData = new ArrayList<LogData>();
    
    public static void addData(String c, Object val) {
        int index = -1;
        for(int i=0; i<logData.size(); i++) {
            int cmp = c.compareTo(logData.get(i).caption);
            if(cmp == 0) {
                index = i;
                break;
            }
        }
        if(index == -1) {
            logData.add(new LogData(c, val));
        }
        else
            logData.get(index).value = val;
    }
    
    public static void addData(String c, String fmt, Object val) {
        addData(c, String.format(fmt, val));
    }
    
    public static void addData(String c, double val) {
        addData(c, String.format("%.3f", val));
    }
    
    public static void addData(String c, int val) {
        addData(c, String.format("%d", val));
    }
    
    private static int consoleMaxSize = 6;
    private static ArrayList<String> consoleOutput = new ArrayList<String>();
    
    public static void log(Object obj) {
        String message = obj.toString();
        if(consoleOutput.size() == 0)
            init_console();
        if(consoleOutput.size() == consoleMaxSize)
            consoleOutput.remove(0);
        consoleOutput.add(message);
    }
    
    private static void init_console() {
        telemetry.addData("\nConsole Output", new Func<String>() {
            @Override
            public String value() {
                String str = "\n";
                for(int i=0; i<consoleOutput.size(); i++)
                    str += consoleOutput.get(i) + "\n";
                for(int i=consoleOutput.size(); i<consoleMaxSize; i++)
                    str += "\n";
                return str;
            }
        });
    }
}

class LogData {
    public String caption;
    public Object value;
    
    public LogData(String c, Object val) {
        caption = c;
        value = val;
        
        telemetry.addData(caption, new Func<String>() {
            @Override
            public String value() {
                return value.toString();
            }
        });
    }
    
    private static Telemetry telemetry;
    
    public static void setTelemetry(Telemetry telemetry) {
        LogData.telemetry = telemetry;
    }
}
