package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Func;


@TeleOp
public class TestMotor extends OpMode {
    DcMotor motor;
    Servo servo;
        
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("flywheel");
        servo = hardwareMap.servo.get("servo");
                
        // Telemetry setup
        telemetry.addData("Power", new Func<String>() {
            @Override
            public String value() { return String.format("%.2f", power); }
        });
        
        telemetry.addData("Deg", new Func<String>() {
            @Override
            public String value() { return String.format("%.2f", (deg*180)); }
        });
        
        telemetry.addData("Deg1", new Func<String>() {
            @Override
            public String value() { return String.format("%.2f", (deg1*180)); }
        });
        
        telemetry.addData("Deg2", new Func<String>() {
            @Override
            public String value() { return String.format("%.2f", (deg2*180)); }
        });
    
        
    }
    
    @Override
    public void init_loop() {
        
    }
    
    @Override
    public void start() {
        
    }
    
    double power = 0.6;
    boolean dpad_state = false;
    
    double deg = 0;
    double deg1 = 0;
    double deg2 = 0;
    
    
    boolean servo_state = false;
    boolean pushButton_state = false;
    boolean servoStatus = false;
        
    @Override
    public void loop() {
        if(gamepad1.x) {
            motor.setPower(power);
        }
        if(gamepad1.a) {
            motor.setPower(0);
        }
        
        if(gamepad1.dpad_up) {
            if(dpad_state == false) {
                power += 0.05;
            }
            dpad_state = true;
        }
        else if(gamepad1.dpad_down) {
            if(dpad_state == false) {
                power -= 0.05;
            }
            dpad_state = true;
        }
        else {
            dpad_state = false;
        }
        
        if(gamepad1.dpad_left) {
            if(servo_state == false) {
                deg -= 1.0/18;
            }
            servo_state = true;
        }
        if(gamepad1.dpad_right) {
            if(servo_state == false) {
                deg += 1.0/18;
            }
            servo_state = true;
        }
        else {
            servo_state = false;
        }
        
        if(deg <0) {deg = 0;}
        else if(deg>180) {deg = 180;}
        
        if(gamepad1.y){
             servo.setPosition(deg);
        }
        
        if(gamepad1.left_stick_button){
             deg1 = deg;
        }
        
        if(gamepad1.right_stick_button){
             deg2 = deg;
        }          
        
        
        /*
        if(gamepad1.b){
             if(pushButton_state == false){
                if(servoStatus == false){
                   servo.setPosition(deg1);
                   servoStatus = true;
                }
                else{
                   servo.setPosition(deg2);
                   servoStatus = false;
                }
             }
        }
        else {
            pushButton_state = false;
        }
        */
        
        if(gamepad1.left_bumper){
            servo.setPosition(deg1);   
        }
        if(gamepad1.right_bumper){
            servo.setPosition(deg2);
        }
        
        if(gamepad1.y){
           servo.setPosition(70.0/180);
           telemetry.addData("Deg","70");
        }
        if(gamepad1.b){
           servo.setPosition(0);
           telemetry.addData("Deg","0");
        }

         telemetry.update();
    }
    
    @Override
    public void stop() {
        motor.setPower(0);
    }
}

