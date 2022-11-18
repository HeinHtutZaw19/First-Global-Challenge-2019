package org.flyingdutchman.ftc.robotcore.util;

public class ComplexNumber {
    public float x,y;
    
    public ComplexNumber(float r, float i) {
        x = r;
        y = i;
    }
    
    public static ComplexNumber newFromPolar(float r, float t) {
        t *= (float)Math.PI/180;
        return new ComplexNumber(r*(float)Math.cos(t), r*(float)Math.sin(t));
    }
    
    public float getMagnitude() {
        return (float)Math.sqrt(x*x + y*y);
    }
    
    public ComplexNumber add(ComplexNumber z) {
        return new ComplexNumber(x+z.x, y+z.y);
    }
    
    public ComplexNumber subtract(ComplexNumber z) {
        return new ComplexNumber(x-z.x, y-z.y);
    }
    
    public ComplexNumber multiply(ComplexNumber z) {
        return new ComplexNumber(x*z.x - y*z.y, x*z.y + y*z.x);
    }
    
    public ComplexNumber divide(ComplexNumber z) {
        float div = z.x*z.x + z.y*z.y;
        return new ComplexNumber((x*z.x + y*z.y) / div, (y*z.x - x*z.y) / div);
    }
    
    public ComplexNumber multiply(float f) {
        return new ComplexNumber(x*f, y*f);
    }
    
    public ComplexNumber divide(float f) {
        return new ComplexNumber(x/f, y/f);
    }
    
    public float atan() {
        return (float)(Math.atan2(y, x) * (180/Math.PI));
    }
    
    @Override
    public String toString() {
        return String.format("%.2f %.2fi", x, y);
    }
}
