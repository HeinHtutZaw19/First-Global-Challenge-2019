package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.*;
import org.flyingdutchman.ftc.robotcore.util.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.io.*;
import java.util.*;


/*
 * Encodes the tick counts to position unit as degree
 * and velocity unit as rad/s.
 *
 * Uses PID algorithms. Position PID and velocity PID.
 */
public class EncoderMotor extends Motor {
    protected double ticksPerRev;
    protected PIDF positionPIDF = new PIDF(0.00547, 0, 0.000298, 0);
    protected PIDF velocityPIDF = new PIDF(0.00176, 0, 0, 0.000227);
    
    protected enum PIDMode {
    	POSITION_PID,
        VELOCITY_PID,
        NO_PID;
    }
    
    protected PIDMode pidMode = PIDMode.NO_PID;
    protected double targetPosition;
    protected double targetVelocity;
    protected double velocityRaw;
    protected double velocity;
    protected MovingAverage filter = new MovingAverage(41);
    
    private String deviceName;
    private int initialTicks;
    
    
    public EncoderMotor(String name) {
        super(name);
        deviceName = name;
        ticksPerRev = motor.getMotorType().getTicksPerRev();
        setPowerChangeRate(25);
        positionPIDF.setTolerance(5);
        velocityPIDF.setTolerance(3);
    }
    
    // ratio = driver/driven
    public void setExternalGearRatio(double ratio) {
        ticksPerRev = motor.getMotorType().getTicksPerRev() * ratio;
        lastPosition = getPosition();
    }
    
    protected enum LimitMode {
    	HAS_LIMIT,
        NO_LIMIT;
    }
    
    protected LimitMode limitMode = LimitMode.NO_LIMIT;
    protected double lowerLimit;
    protected double upperLimit;
    protected double limitClearance = 5;
    
    public void setLimit(double lower, double upper) {
        limitMode = LimitMode.HAS_LIMIT;
        lowerLimit = lower;
        upperLimit = upper;
    }
    
    // Position in degrees
    public void setPosition(double deg) {
        pidMode = PIDMode.POSITION_PID;
        if(limitMode == LimitMode.HAS_LIMIT) {
            if(deg < lowerLimit || deg > upperLimit) {
                Monitor.log("****************");
                Monitor.log("Target out of bounds.");
                Monitor.log(String.format("setPosition(%.2f)", deg));
                Monitor.log("****************");
                if(deg < lowerLimit)
                    targetPosition = lowerLimit;
                else if(deg > upperLimit)
                    targetPosition = upperLimit;
            }
            else
                targetPosition = deg;
        }
        else {
            double err, err1, err2, err3;
            err1 = (deg - getPosition()) % 360;
            err2 = err1 - 360;
            err3 = err1 + 360;
            if(Math.abs(err1) < Math.abs(err2)) {
                if(Math.abs(err1) < Math.abs(err3))
                    err = err1;
                else
                    err = err3;
            }
            else if(Math.abs(err2) < Math.abs(err3))
                err = err2;
            else
                err = err3;
            
            targetPosition = getPosition() + err;
        }
    }
    
    public double getPosition() {
        return 360 * ((motor.getCurrentPosition()+initialTicks) / ticksPerRev);
    }
    
    public void move(double deg) {
        pidMode = PIDMode.POSITION_PID;
        targetPosition = getPosition() + deg;
    }
    
    
    // Gives encoder motor a position sequence to move
    private Step positionSteps = new Step(this) {
        @Override
        protected void func(Operation device, double val) {
            EncoderMotor motor = (EncoderMotor) device;
            motor.setPosition(val);
        }
    };
    
    public void setNextPosition(double pos) {
        positionSteps.next(pos);
    }
    
    
    // Velocity in rad/s
    public void setVelocity(double v) {
        pidMode = PIDMode.VELOCITY_PID;
        targetVelocity = v * (180 / Math.PI);
    }
    
    public double getVelocity() {
        return velocity * (Math.PI / 180);
    }
    
    public double getVelocityRaw() {
        return velocityRaw * (Math.PI / 180);
    }
    
    public void setPositionPID(double p, double i, double d) {
        positionPIDF.setCoefficients(p, i, d, 0);
    }
    
    public void setPositionPIDF(double p, double i, double d, double f) {
        positionPIDF.setCoefficients(p, i, d, f);
    }
    
    public void setVelocityPIDF(double p, double i, double d, double f) {
        velocityPIDF.setCoefficients(p, i, d, f);
    }
    
    protected double lastPosition = 0;
    
    @Override
    public void start() {
        lastPosition = getPosition();
    }
    
    @Override
    public void loop(double dt) {
        // Velocity measurement.
        double currentPosition = getPosition();
        velocityRaw = (currentPosition - lastPosition) / dt;
        velocity = filter.getSmoothened(velocityRaw);
        
        // Position PID
       	if(pidMode == PIDMode.POSITION_PID) {
            double pidOut = positionPIDF.getPIDControllerOutput(
                targetPosition, currentPosition, dt
            );
            double pow, ff;
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
            pow = pidOut + ff;

            super.setPower(pow);
        }
        
        // Velocity PID
        else if(pidMode == PIDMode.VELOCITY_PID) {
            double pow;
            if(Math.abs(targetVelocity) < 3 && velocityPIDF.isReached())
                pow = 0;
            else {
                pow = velocityPIDF.getControllerOutput(
                    targetVelocity, velocityRaw, dt
                );
            }
            super.setPower(pow);
        }
        
        // Limit constrain function
        if(limitMode == LimitMode.HAS_LIMIT) {
            if((currentPosition<lowerLimit + limitClearance &&
                powerTransition.getTarget() < 0))
            {
                double perc = (currentPosition - lowerLimit) / limitClearance;
                double pow = Math.max(-perc,
                             Math.min(powerTransition.getTarget(), perc));
                powerTransition.setTarget(pow);
            }
            else if((currentPosition>upperLimit - limitClearance &&
                     powerTransition.getTarget() > 0))
            {
                double perc = (upperLimit - currentPosition) / limitClearance;
                double pow = Math.max(-perc,
                             Math.min(powerTransition.getTarget(), perc));
                powerTransition.setTarget(pow);
            }
        }
        
        super.loop(dt);
        lastPosition = currentPosition;
    }
    
    @Override
    public boolean isBusy() {
       	if(pidMode == PIDMode.POSITION_PID)
            return !positionPIDF.isReached();
       	else if(pidMode == PIDMode.VELOCITY_PID) {
            if(OpModeEx.stopRequest)
                return false;
            else
                return !velocityPIDF.isReached();
        }
        else
            return super.isBusy();
    }
    
    @Override
    public void setPower(double p) {
        pidMode = PIDMode.NO_PID;
        powerTransition.setTarget(p);
    }
    
    
    /*
     *
     *
     * Do not use the codes below !!!
     * Bugged and is not maintained.
     *
     *
     */
    
    
    /*
     * Data storage saves encoder position after power off.
     * Simple file io operations.
     */
    /*private static File memoryFile = new File(OpModeEx.home + "encoder");
    
    private static ArrayList<EncoderMotor> encoders = null;
    private static ArrayList<Pair<String, Integer>> encoderMemory = null;
    
    public void enableMemory() {
        if(encoderMemory == null) {
            encoders = new ArrayList<EncoderMotor>();
            encoderMemory = new ArrayList<Pair<String, Integer>>();
            loadEncoderMemory();
            OpModeEx.addOperation(new Operation() {
                @Override
                public void start() {}
                
                @Override
                public void stop() {
                    encoders.clear();
                    encoderMemory.clear();
                    encoders = null;
                    encoderMemory = null;
                }
                
                @Override
                public void loop(double dt) {
                    for(int i=0; i<encoders.size(); i++) {
                        EncoderMotor e = encoders.get(i);
                        saveEncoderMemory(
                            e.deviceName,
                            e.motor.getCurrentPosition() + e.initialTicks
                        );
                    }
                    updateEncoderMemory();
                }
                
                @Override
                public boolean isBusy() { return false; }
            }, 0);
        }
        
        encoders.add(this);
        initialTicks = retriveEncoderMemory(deviceName);
    }
    
    private static void saveEncoderMemory(String name, int data) {
        for(int i=0; i<encoderMemory.size(); i++) {
            if(name.compareTo(encoderMemory.get(i).getLeft()) == 0) {
                encoderMemory.get(i).setRight(data);
                return;
            }
        }
        encoderMemory.add(new Pair<String, Integer> (name, data));
    }
    
    private static int retriveEncoderMemory(String name) {
        for(int i=0; i<encoderMemory.size(); i++) {
            if(name.compareTo(encoderMemory.get(i).getLeft()) == 0)
                return encoderMemory.get(i).getRight();
        }
        return 0;
    }
    
    private static void updateEncoderMemory() {
        try {
            FileWriter fw = new FileWriter(memoryFile);
            BufferedWriter bw = new BufferedWriter(fw);
            
            for(int i=0; i<encoderMemory.size(); i++) {
                bw.write(encoderMemory.get(i).getLeft());
                bw.write(' ');
                bw.write(String.valueOf(encoderMemory.get(i).getRight()));
                bw.newLine();
            }
            bw.flush();
            bw.close();
        }
        catch(IOException e) {
            Monitor.log(e);
        }
    }
    
    private static void loadEncoderMemory() {
        try {
            FileReader fr = new FileReader(memoryFile);
            BufferedReader br = new BufferedReader(fr);
            
            String line;
            while((line=br.readLine()) != null) {
                StringTokenizer st = new StringTokenizer(line);
                String name = st.nextToken(); 
                int data = Integer.parseInt(st.nextToken());
                encoderMemory.add(new Pair<String, Integer> (name, data));
            }
            fr.close();
            br.close();
        }
        catch(FileNotFoundException e) {
            try {
                new File(OpModeEx.home).mkdir();
                memoryFile.createNewFile();
            }
            catch(IOException ex) {
                Monitor.log(ex);
            }
        }
        catch(Exception e) {
            Monitor.log(e);
        }
    }*/
}
