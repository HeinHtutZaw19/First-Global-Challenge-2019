package org.flyingdutchman.ftc.robotcore.util;

public class Vector2f {
    public float x, y;
    
    public Vector2f(float x, float y) {
        this.x = x;
        this.y = y;
    }
    
    public float getMagnitude() {
        return (float)Math.sqrt(x*x + y*y);
    }
    
    public Vector2f add(Vector2f v) {
        return new Vector2f(x+v.x, y+v.y);
    }
    
    public Vector2f subtract(Vector2f v) {
        return new Vector2f(x-v.x, y-v.y);
    }
    
    public Vector2f multiply(float f) {
        return new Vector2f(x*f, y*f);
    }
    
    public Vector2f divide(float f) {
        return new Vector2f(x/f, y/f);
    }
    
    public float dot(Vector2f v) {
        return x*v.x + y*v.y;
    }
    
    public float cross(Vector2f v) {
        return x*v.y - v.x*y;
    }
    
    public Vector2f cross(float f) {
        return new Vector2f(y*f, -x*f);
    }
    
    public Vector2f rotate(float t) {
        t = (float)Math.PI/180;
        return new Vector2f(x*(float)Math.cos(t) - y*(float)Math.sin(t),
                            x*(float)Math.sin(t) + y*(float)Math.cos(t));
    }
    
    @Override
    public String toString() {
        return String.format("%.3fi %.3fj", x, y);
    }
}
