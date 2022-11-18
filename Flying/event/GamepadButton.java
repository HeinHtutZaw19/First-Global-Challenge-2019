package org.flyingdutchman.ftc.robotcore.event;

import org.flyingdutchman.ftc.robotcore.util.Pair;
import java.util.ArrayList;


public class GamepadButton {
    boolean lastState = false;
    
    public enum Key {
        X, Y, A, B, L1, R1, L2, R2, L3, R3, UP, DOWN, RIGHT, LEFT;
    }
    
    Key key;
    
    public GamepadButton(Key k) {
        key = k;
        buttonList.add(this);
    } 
    
    
    public boolean getState() {
        switch(key) {
          case X:
            return OpModeEx.gamepad.x;
          
          case Y:
            return OpModeEx.gamepad.y;
          
          case A:
            return OpModeEx.gamepad.a;
          
          case B:
            return OpModeEx.gamepad.b;
          
          case L2:
            return OpModeEx.gamepad.left_trigger;
          
          case R2:
            return OpModeEx.gamepad.right_trigger;
          
          case L1:
            return OpModeEx.gamepad.left_bumper;
          
          case R1:
            return OpModeEx.gamepad.right_bumper;
          
          case L3:
            return OpModeEx.gamepad.left_stick_button;
          
          case R3:
            return OpModeEx.gamepad.right_stick_button;
          
          case UP:
            return OpModeEx.gamepad.dpad_up;
          
          case DOWN:
            return OpModeEx.gamepad.dpad_down;
          
          case RIGHT:
            return OpModeEx.gamepad.dpad_right;
          
          case LEFT:
            return OpModeEx.gamepad.dpad_left;
          
          default:
            return false;
        }
    }
    
    private boolean risingEdge = false;
    private boolean fallingEdge = false;
    
    public boolean onPressed() { 
        boolean ret = risingEdge;
        risingEdge = false;
        return ret;
    }
    
    public boolean onReleased() { 
        boolean ret = fallingEdge;
        fallingEdge = false;
        return ret;
    }
    
    private static ArrayList<GamepadButton> buttonList = new ArrayList<>();
    protected static ArrayList<ComboPair> comboPairs = new ArrayList<>();
    protected static boolean comboEnabled = false;
    
    public static void enableCombo() { comboEnabled = true; }
    
    public static void loop() {
        for(int i=0; i<buttonList.size(); i++) {
            GamepadButton btn = buttonList.get(i);
            
            boolean curState = btn.getState();
            
            if(curState == true && btn.lastState == false) {
                btn.risingEdge = true;
                btn.fallingEdge = false;
                if(comboEnabled) {
                    for(int j=0; j<comboPairs.size(); j++)
                        comboPairs.get(j).checkCombo(btn.key);
                }
            }
            
            if(curState == false && btn.lastState == true) {
                btn.fallingEdge = true;
                btn.risingEdge = false;
            }
            
            btn.lastState = curState;
        }
    }
    
    public static class ComboPair {
        private Key key1, key2;
        private Key next;
        private double t1;
        private int step = 0;
        private boolean comboTriggered = false;
        
        public ComboPair(Key k1, Key k2) {
            key1 = k1;
            key2 = k2;
            GamepadButton.comboPairs.add(this);
        }
        
        public void checkCombo(Key k) {
            if(step == 1) {
                if(k == next) {
                    double delay = OpModeEx.now() - t1;
                    if(delay < 0.3)
                        comboTriggered = true;
                    step = 0;
                }
            }
            if(step == 0) {
                if(k == key1 || k == key2) {
                    next = k == key1 ? key2 : key1;
                    t1 = OpModeEx.now();
                    step = 1;
                }
            }
        }
        
        public boolean isTriggered() {
            double delay = OpModeEx.now() - t1;
            boolean ret;
            ret = delay < 0.3 ? comboTriggered : false;
            comboTriggered = false;
            return ret;
        }
    }
}
