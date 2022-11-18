package org.flyingdutchman.ftc.robotcore.util;

public class Vector3f {
    public float x, y, z;
    
    public Vector3f(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    public Vector3f(Vector2f v) {
        x = v.x;
        y = v.y;
        z = 0;
    }
    
    public float getMagnitude() {
        return (float)Math.sqrt(x*x + y*y + z*z);
    }
    
    public Vector3f add(Vector3f v) {
        return new Vector3f(x+v.x, y+v.y, z+v.z);
    }
    
    public Vector3f subtract(Vector3f v) {
        return new Vector3f(x-v.x, y-v.y, z-v.z);
    }
    
    public Vector3f multiply(float f) {
        return new Vector3f(x*f, y*f, z*f);
    }
    
    public Vector3f divide(float f) {
        return new Vector3f(x/f, y/f, z/f);
    }
    
    public float dot(Vector3f v) {
        return x*v.x + y*v.y + z*v.z;
    }
    
    @Override
    public String toString() {
        return String.format("%.3fi %.3fj %.3fk", x, y, z);
    }
}
