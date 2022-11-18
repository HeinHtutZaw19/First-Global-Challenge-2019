package org.flyingdutchman.ftc.robotcore.hardware;

import org.flyingdutchman.ftc.robotcore.event.OpModeEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class CServo {
    private CRServo servo;
    
    public CServo(String name) {
        servo = OpModeEx.hardware.get(CRServo.class, name);
    }
    
    public void setDirection(Direction dir) {
        servo.setDirection(dir);
    }
    
    public void setPower(double p) {
        servo.setPower(p);
    }
}

