package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.Operation;
import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import org.flyingdutchman.ftc.robotcore.util.Direction;
import org.flyingdutchman.ftc.robotcore.util.Vector2f;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.*;


public class IMU implements Operation {
    private BNO055IMU imu;
    
    public IMU(String name) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        
        BNO055IMU.CalibrationData calibration = new BNO055IMU.CalibrationData();
        calibration.dxAccel = 1;
        calibration.dxGyro = -1;
        calibration.dxMag = -221;
        calibration.dyAccel = -29;
        calibration.dyGyro = 1;
        calibration.dyMag = -80;
        calibration.dzAccel = -31;
        calibration.dzGyro = 0;
        calibration.dzMag = -314;
        calibration.radiusAccel = 1000;
        calibration.radiusMag = 922;
        parameters.calibrationData = calibration;
        
        imu = OpModeEx.hardware.get(BNO055IMU.class, name);
        imu.initialize(parameters);
        OpModeEx.addOperation(this, 0);
    }
    
    private float lastHeading;
    private float heading;
    private float angularVelocity;
    private float hdiff;
    
    public float getAngularVelocity() { return angularVelocity; }
    
    public float getHeading() { return Direction.convert(heading); }
    
    public void setHeading(float t) {
        hdiff += (t - heading);
        heading = t;
    }
    
    @Override
    public void start() {
        lastHeading = imu.getAngularOrientation(
                          AxesReference.INTRINSIC,
                          AxesOrder.ZYX,
                          AngleUnit.DEGREES
                      ).firstAngle;
    }
    
    @Override
    public void stop() {}
    
    @Override
    public void loop(double dt) {
        heading = imu.getAngularOrientation(
                      AxesReference.INTRINSIC,
                      AxesOrder.ZYX,
                      AngleUnit.DEGREES
                  ).firstAngle + hdiff;
        float dtheta = heading - lastHeading;
        angularVelocity = dtheta / (float)dt;
        lastHeading = heading;
    }
    
    @Override
    public boolean isBusy() { return false; }
}
