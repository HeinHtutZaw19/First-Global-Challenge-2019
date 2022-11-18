package org.flyingdutchman.ftc.robotcore.event;

public interface Operation {
    public abstract void start();
    
    public abstract void stop();
    
    public abstract void loop(double dt);
    
    public abstract boolean isBusy();
}
