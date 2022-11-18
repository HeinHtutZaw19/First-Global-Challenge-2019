package org.flyingdutchman.ftc.robotcore.util;

import java.util.LinkedList;
import java.util.Queue;

/*
 * Real time signal smoothing using moving average algorithm.
 */
public class MovingAverage {
    private int filterWidth;
    
    public MovingAverage(int width) {
        filterWidth = width;
        box = new LinkedList<>();
        for(int i=0; i<filterWidth; i++)
            box.add(0.0);
    }
    
    private Queue<Double> box;
    private double Ey = 0;
    
    public double getSmoothened(double y) {
        Ey -= box.remove();
        Ey += y;
        box.add(y);
        return Ey / filterWidth;
    }
}
