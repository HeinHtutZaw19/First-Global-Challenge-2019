package org.flyingdutchman.ftc.robotcore.util;

public class Direction {
    private float val;
    
    public Direction(float t) {
        val = t;
    }
    
    public float getValue() {
        return val;
    }
    
    public float getError(Direction dir) {
        return convert(val - dir.val);
    }
    
    public static float convert(float t) {
        float a, b, c;
        a = t % 360;
        b = a - 360;
        c = a + 360;
        if(Math.abs(a) < Math.abs(b)) {
            if(Math.abs(a) < Math.abs(c))
                t = a;
            else
                t = c;
        }
        else if(Math.abs(b) < Math.abs(c))
            t = b;
        else
            t = c;
        return t;
    }
    
    public static float linearMap(Direction d1, Direction d2, float a) {
        float diff = d2.getError(d1);
        return d1.val + diff*a;
    }
}
