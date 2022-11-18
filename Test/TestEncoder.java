package org.flyingdutchman.ftc.teamcode2019;

import org.flyingdutchman.ftc.robotcore.*;
import org.flyingdutchman.ftc.robotcore.event.*;
import org.flyingdutchman.ftc.robotcore.hardware.*;
import org.flyingdutchman.ftc.robotcore.event.GamepadButton.Key;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestEncoder extends OpModeEx {
    EncoderMotor motor;
    Motor motorTest;
    AnalogInput pot1, pot2, pot3, pot4;
    DigitalInput btnMotorEn;
    double Kp, Ki, Kd, Kf, setPoint, Kpow, motorTest_power,targetPower ;
    GamepadButton startButton = new GamepadButton(Key.A);
    
            
    @Override
    public void init() {
        super.init();
        motor = new EncoderMotor("flywheel");
        motorTest = new Motor("right");
        motor.setExternalGearRatio(1.0/2);
        //motor.setExternalGearRatio(1.0);
        
        
        pot1 = new AnalogInput("pot1");
        pot2 = new AnalogInput("pot2");
        pot3 = new AnalogInput("pot3");
        pot4 = new AnalogInput("pot4");
        
        
        //motor.setLimit(0, 390);
    }
    
    
    @Override
    public void loop() {
        Monitor.addData("Positon", String.format("%.2f deg", motor.getPosition()));
        Monitor.addData("Velocity", String.format("%.2f rad/s", motor.getVelocity()));
       /* if(gamepad1.b) {
            motor.setPower(0);
        }
        if(gamepad1.y) {
            motor.move(60);
        }
        if(gamepad1.x) {
            motor.move(120);
        }
        if(gamepad1.a) {
            motor.move(180);
        }*/
        
        boolean btnState = startButton.getState();
        
        
        if(btnState == true) {
            motor.setVelocityPIDF(Kp, Ki, Kd, Kf);
            motor.setVelocity(setPoint);
            motorTest.setPower(motorTest_power);
        }
        
        else {
            
            setPoint = -gamepad1.left_stick_y * 35;
            Kp = /*0.001518;*/ pot4.getInput() * 0.003;
            Ki = pot3.getInput() * 0;
            Kd = pot2.getInput() * 0;
            Kf = 0.000227; //pot1.getInput() * 0.05;
            motorTest_power = setPoint*(180/Math.PI) * Kf;
            
            motor.setPower(0);
            motorTest.setPower(0);
        }
        
        
        /*
        if(gamepad1.b) {
            motor.setPower(0);
        }
        if(gamepad1.y) {
            motor.setPower(0.5);
        }
        if(gamepad1.x) {
            motor.setPower(-0.6);
        }
        if(gamepad1.a) {
            motor.setPower(1);
        }
        */
        Monitor.addData("Kp", Kp);
        Monitor.addData("Ki", Ki);
        Monitor.addData("Kd", Kd);
        Monitor.addData("Kf", Kf);
        Monitor.addData("Setpoint(rad/s)",setPoint);
        Monitor.addData("Motor Power",motorTest_power);
        super.loop();
    }
    
    @Override
    public void stop() {
        Monitor.reset();
        motor.setPower(0);
        super.stop();
    }
}

