package org.flyingdutchman.ftc.teamcode2019;

import org.flyingdutchman.ftc.robotcore.*;
import org.flyingdutchman.ftc.robotcore.event.*;
import org.flyingdutchman.ftc.robotcore.event.GamepadButton.Key;
import org.flyingdutchman.ftc.robotcore.event.GamepadButton.ComboPair;
import org.flyingdutchman.ftc.robotcore.hardware.*;
import org.flyingdutchman.ftc.robotcore.util.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(name = "Flying Dutchman")
public class FlyingOpMode extends OpModeEx {
    TwoWheelDrive driveSystem;
    IMU imu;
    VuforiaWebcam webcam;
    AutoParking autoParking;
    
    EncoderMotor flywheel;
    Motor belt1;
    Motor belt2;
    TwinLinearMotor lift;
    
    Servo flywheelServo;
    Servo pushServo;
    Servo dropServo;
    Servo extensionServo1;
    Servo extensionServo2;
    CServo cservo1;
    CServo cservo2;
    
    GamepadButton runButton =       new GamepadButton(Key.R1);
    GamepadButton directionInvert = new GamepadButton(Key.L1);
    GamepadButton flywheelSwitch =  new GamepadButton(Key.B);
    GamepadButton flywheelShoot =   new GamepadButton(Key.A);
    GamepadButton belt1Switch =     new GamepadButton(Key.X);
    GamepadButton belt2Switch =     new GamepadButton(Key.Y);
    GamepadButton ballLift =        new GamepadButton(Key.L2);
    GamepadButton ballDrop =        new GamepadButton(Key.R2);
    GamepadButton btnUp =           new GamepadButton(Key.UP);
    GamepadButton btnDown =         new GamepadButton(Key.DOWN);
    GamepadButton btnLeft =         new GamepadButton(Key.LEFT);
    GamepadButton btnRight =        new GamepadButton(Key.RIGHT);
    GamepadButton btnL3 =           new GamepadButton(Key.L3);
    GamepadButton btnR3 =           new GamepadButton(Key.R3);
    ComboPair     belt1Forward =    new ComboPair(Key.UP,   Key.X);
    ComboPair     belt2Forward =    new ComboPair(Key.UP,   Key.Y);
    ComboPair     belt1Backward =   new ComboPair(Key.DOWN, Key.X);
    ComboPair     belt2Backward =   new ComboPair(Key.DOWN, Key.Y);
    ComboPair     forcedShoot =     new ComboPair(Key.UP,   Key.A);
    ComboPair     autoBallDrop =    new ComboPair(Key.UP,   Key.R2);
    ComboPair     autoModeCombo =   new ComboPair(Key.L3,   Key.R3);
    ComboPair     collectorSwitch = new ComboPair(Key.RIGHT,Key.X);
    
    boolean directionInverted = false;
    boolean flywheelState = false;
    boolean belt1State = false;
    boolean belt2State = false;
    boolean cservoState = false;
    boolean dropState = false;
    boolean pushState = false;
    boolean liftManual = false;
    boolean autoEnabled = false;
    
    final double TOP_SPEED = 13;
    final double BELT1_POWER = 0.7;
    final double BELT2_POWER = 0.7;
    final double FLYWHEEL_SPEED = 51.5;
    final double FLYWHEEL_SERVO_POSITION1 = 28.5;
    final double FLYWHEEL_SERVO_POSITION2 = 0;
    final double PUSH_SERVO_POSITION1 = 0;
    final double PUSH_SERVO_POSITION2 = 130;
    final double DROP_SERVO_POSITION1 = 75;
    final double DROP_SERVO_POSITION2 = 22.5;
    final double EXTENSION_SERVO1_POSITION = 75;
    final double EXTENSION_SERVO2_POSITION = 30;
    
    final int TEAM_RED = 0;
    final int TEAM_BLUE = 1;
    
    final String RED_TARGET_IMAGE_NAME = "World_Map";
    final String BLUE_TARGET_IMAGE_NAME = "World_Map";
    final Vector3f RED_TARGET_IMAGE_POSITION = new Vector3f(300, 302, 726);
    final Vector3f BLUE_TARGET_IMAGE_POSITION = new Vector3f(300, 398, 726);
    final Vector2f RED_PROJECTILE_TARGET = new Vector2f(300, 340);
    final Vector2f BLUE_PROJECTILE_TARGET = new Vector2f(300, 360);
    final float PROJECTILE_RANGE = 214.6f;
    
    @Override
    public void init() {
        super.init();
        
        driveSystem = new TwoWheelDrive("left", "right");
        driveSystem.setMaximumSpeed(TOP_SPEED);
        driveSystem.setVelocityPIDF(0.00176, 0.000005, 0, 0.000227);
        driveSystem.setPositionPID(0.01, 0.000005, 0);
        driveSystem.setPhysicalParameters(
            4.5f,
            45+(2.1f*2),
            0
        );
        imu = new IMU("imu");
        webcam = new VuforiaWebcam("Webcam 1");
        webcam.setCameraPosition(
            new Vector3f(-6.5f, -10.5f, 36),
            15
        );
        webcam.addTargetImage(
            RED_TARGET_IMAGE_NAME,
            RED_TARGET_IMAGE_POSITION,
            -90
        );
        /*webcam.addTargetImage(
            BLUE_TARGET_IMAGE_NAME,
            BLUE_TARGET_IMAGE_POSITION,
            90
        );*/
        autoParking = new AutoParking(driveSystem, imu, webcam);
        autoParking.setToolPosition(new Vector2f(1, -10.5f)); // z=31
        autoParking.setDistancePID(0.4, 0, 0);
        autoParking.setDirectionPID(2.5, 0, 0);
        
        flywheel = new EncoderMotor("flywheel");
        flywheel.setExternalGearRatio(2/12.5);
        flywheel.setPowerChangeRate(10);
        flywheel.setVelocityPIDF(0.000130,0.00000055,0.0000026065,0.000115);
        //flywheel.setVelocityPIDF(0.000208,0.000,0.000002,0.0001);
        belt1 = new Motor("belt1");
        belt1.setDirection(Direction.REVERSE);
        belt2 = new Motor("belt2");
        lift = new TwinLinearMotor("lift1", "lift2");
        lift.setDirection(Direction.REVERSE, Direction.FORWARD);
        lift.setLengthPerRev(16.2 / 6.933);
        lift.setLimit(0, 31);
        lift.setPowerChangeRate(10);
        lift.setAdjustingPID(0, 0, 0);
        
        flywheelServo = new Servo("shoot");
        flywheelServo.setLimit(-90, 90);
        pushServo = new Servo("pushServo");
        pushServo.setLimit(0, 180);
        dropServo = new Servo("drop1");
        dropServo.setLimit(0, 90);
        cservo1 = new CServo("cservo1");
        cservo2 = new CServo("cservo2");
        cservo1.setDirection(Direction.REVERSE);
        extensionServo1 = new Servo("extension1");
        extensionServo2 = new Servo("extension2");
        
        updateConfigurationString();
        Monitor.log("Use arrow buttons to config Flying Dutchman");
    }
    
    int team = 0;
    String[] team_names = {"Red", "Blue"};
    int position = 1;
    int config_index = 0;
    
    @Override
    public void init_loop() {
        if(btnUp.onPressed()) {
            config_index = (config_index-1+2) % 2;
            updateConfigurationString();
        }
        if(btnDown.onPressed()) {
            config_index = (config_index+1) % 2;
            updateConfigurationString();
        }
        if(btnLeft.onPressed()) {
            switch(config_index) {
              case 0:
                team = (team-1+2) % 2;
                break;
              case 1:
                position = (position-1-1+3) % 3 + 1;
                break;
            }
            updateConfigurationString();
        }
        if(btnRight.onPressed()) {
            switch(config_index) {
              case 0:
                team = (team+1) % 2;
                break;
              case 1:
                position = (position+1-1) % 3 + 1;
                break;
            }
            updateConfigurationString();
        }
        super.init_loop();
    }
    
    public void updateConfigurationString() {
        if(config_index == 0)
            Monitor.addData("Team", "<<%s>>", team_names[team]);
        else
            Monitor.addData("Team", team_names[team]);
        
        if(config_index == 1)
            Monitor.addData("Crew position", "<<%d>>", position);
        else
            Monitor.addData("Crew position", position);
    }
    
    @Override
    public void start() {
        config_index = -1;
        updateConfigurationString();
        
        float initial_heading = 0;
        Vector2f initial_position = new Vector2f(0, 0);
        switch(team) {
          case TEAM_RED:
            initial_heading = 90;
            switch(position) {
                case 1:
                    initial_position = new Vector2f(100, 40);
                    break;
                case 2:
                    initial_position = new Vector2f(300, 40);
                    break;
                case 3:
                    initial_position = new Vector2f(500, 40);
                    break;
            }
            break;
          case TEAM_BLUE:
            initial_heading = -90;
            switch(position) {
                case 1:
                    initial_position = new Vector2f(100, 660);
                    break;
                case 2:
                    initial_position = new Vector2f(300, 660);
                    break;
                case 3:
                    initial_position = new Vector2f(500, 660);
                    break;
            }
            break;
        }
        imu.setHeading(initial_heading);
        driveSystem.setPosition(initial_position);
        autoParking.setTarget(new Vector2f(300, 350), PROJECTILE_RANGE);
        
        flywheelServo.setPosition(FLYWHEEL_SERVO_POSITION1);
        pushServo.setPosition(PUSH_SERVO_POSITION1);
        dropServo.setPosition(DROP_SERVO_POSITION1);
        /*cservoState = true;
        cservo1.setPower(1);
        cservo2.setPower(1);*/
        extensionServo1.setPosition(EXTENSION_SERVO1_POSITION);
        extensionServo2.setPosition(EXTENSION_SERVO2_POSITION);
        super.start();
        Monitor.log("Flying Dutchman is online");
    }
    
    @Override
    public void loop() {
        // Left joystick input for move function
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double val = Math.max(Math.abs(x), Math.abs(y));
        double dir = Math.atan2(x, -y) * (180/Math.PI);
         /*
        double Kp = pot4.getInput() * 0.0002;
        double Ki = pot3.getInput() * 0.000003 ;
        double Kd = pot2.getInput() * 0.000005 ;
        double Kf = 0.000112;
        flywheel.setVelocityPIDF(Kp, Ki, Kd, Kf);
        
        Monitor.addData("Kp", "%.7f", Kp);
        Monitor.addData("Ki", "%.7f", Ki);
        Monitor.addData("Kd", "%.7f", Kd);
        Monitor.addData("Kf", "%.7f", Kf);*/
        
        if(autoEnabled == false) {
            // Press R1 for speed
            if(runButton.getState() == false)
                val = val * 0.6;
            
            // Press L1 to invert direction
            if(directionInvert.onPressed())
                directionInverted = !directionInverted;
            
            if(directionInverted)
                dir = Direction.convert(dir + 180);
            Monitor.addData("Joystick input", String.format("%.2f, %.2f", val*TOP_SPEED, dir));
            driveSystem.drive(val*TOP_SPEED, dir);
        }
        else {
            if(val > 0.8) {
                Monitor.log("Automatic aiming disabled");
                autoParking.setAutoEnabled(false);
                autoEnabled = false;
            }
        }
        
        // Press L3 + R3 for automatic functions
        if(autoModeCombo.isTriggered()) {
            Monitor.log("Automatic aiming enabled");
            if(autoEnabled == false) {
                Vector2f target = null;
                switch(team) {
                  case TEAM_RED:
                    target = RED_PROJECTILE_TARGET;
                    break;
                  case TEAM_BLUE:
                    target = BLUE_PROJECTILE_TARGET;
                    break;
                }
                autoParking.setTarget(target, PROJECTILE_RANGE);
                autoParking.setAutoEnabled(true);
                autoEnabled = true;
            }
        }
        
        // Press B to turn on flywheel
        if(flywheelSwitch.onPressed()) {
            flywheelState = !flywheelState;
            if(flywheelState == true) {
                flywheel.setVelocity(FLYWHEEL_SPEED);
                Monitor.log("Flywheel switch on");
            }
            else {
                flywheel.setPower(0);
                Monitor.log("Flywheel switch off");
            }
        }
        
        // Press A to shoot small ball
        // Press UP+A to force shoot
        if(flywheelShoot.onPressed()) {
            double speed_err = FLYWHEEL_SPEED - flywheel.getVelocity();
            if(Math.abs(speed_err) < FLYWHEEL_SPEED * 0.04 
               || forcedShoot.isTriggered())
            {
                if(flywheelServo.isBusy() == false) {
                    flywheelServo.setPosition(FLYWHEEL_SERVO_POSITION2);
                    flywheelServo.setNextPosition(FLYWHEEL_SERVO_POSITION1,
                                                  0.5);
                    Monitor.log("Shoot");
                }
            }
            else if(flywheelState == false) {
                Monitor.log("Flywheel not powered");
                Monitor.log("Press B to switch on");
            }
            else {
                Monitor.log("Flywheel not fast enough. Wait for it to power.");
            }
        }
        
        // Press X to turn on belt 1
        if(belt1Switch.onPressed()) {
            if(belt1Forward.isTriggered()) {
                belt1State = true;
                belt1.setPower(BELT1_POWER);
                if(cservoState == true) {
                    cservo1.setPower(1);
                    cservo2.setPower(1);
                }
                Monitor.log("Belt-1 switch on");
            }
            else if(belt1Backward.isTriggered()) {
                belt1State = true;
                belt1.setPower(-BELT1_POWER);
                if(cservoState == true) {
                    cservo1.setPower(-1);
                    cservo2.setPower(-1);
                }
                Monitor.log("Dispencing micro-pollutants");
            }
            else if(collectorSwitch.isTriggered()) {
                cservoState = !cservoState;
                if(cservoState == true) {
                    cservo1.setPower(1);
                    cservo2.setPower(1);
                    Monitor.log("Collector switch on");
                }
                else {
                    cservo1.setPower(0);
                    cservo2.setPower(0);
                    Monitor.log("Collector switch off");
                }
            }
            else {
                belt1State = !belt1State;
                if(belt1State == true) {
                    belt1.setPower(BELT1_POWER);
                    Monitor.log("Belt-1 switch on");
                }
                else {
                    belt1.setPower(0);
                    if(cservoState == true) {
                        cservo1.setPower(1);
                        cservo2.setPower(1);
                    }
                    Monitor.log("Belt-1 switch off");
                }
            }
        }
        
        // Press Y to turn on belt 2
        if(belt2Switch.onPressed()) {
            ballDeployStop();
            if(belt2Forward.isTriggered()) {
                belt2State = true;
                belt2.setPower(BELT2_POWER);
                Monitor.log("Belt-2 moving macro-pollutant upward");
            }
            else if(belt2Backward.isTriggered()) {
                belt2State = true;
                belt2.setPower(-BELT2_POWER);
                Monitor.log("Belt-2 moving macro-pollutant downward");
            }
            else {
                belt2State = !belt2State;
                if(belt2State == true) {
                    belt2.setPower(BELT2_POWER);
                Monitor.log("Belt moving macro-pollutant upward");
                }
                else {
                    belt2.setPower(0);
                    Monitor.log("Belt-2 switch off");
                }
            }
        }
        
        
        // L2 for push Servo
        if(ballLift.onPressed()) {
            ballDeployStop();
            pushServo.setPosition(PUSH_SERVO_POSITION2);
            pushServo.setNextPosition(PUSH_SERVO_POSITION1, 0.7);
            Monitor.log("Pushed macro-pollutant into container");                
        }
        
        if(ballDrop.onPressed()) {
            // UP+R2 for auto ball deploy
            if(autoBallDrop.isTriggered()) {
                ballDeployStart();
                Monitor.log("Quick deploy of macro-pollutants");
            }
            
            // R2 to drop ball
            else {
                dropState = !dropState;
                ballDeployStop();
                dropServoReturn.clear();
                if(dropState == true) {
                    dropServo.setPosition(DROP_SERVO_POSITION2);
                    dropServoReturn.next(0, 3);
                    Monitor.log("Ball dropped");
                }
                else {
                    dropServo.setPosition(DROP_SERVO_POSITION1);
                    Monitor.log("Drop servo back in position");
                }
            }
        }
                
        ballDeployLoop();
        
        double lift_ctrl = -gamepad1.right_stick_y;
        if(Math.abs(lift_ctrl) > 0.1) {
            lift.setPower(lift_ctrl);
            liftManual = true;
        }
        else if(liftManual == true) {
            lift.setPower(0);
            liftManual = false;
        }
        
        Monitor.addData("flywheel velocity",flywheel.getVelocity());
        Monitor.addData("Lift 1 position", lift.getPosition1());
        Monitor.addData("Lift 2 position", lift.getPosition2());
        Monitor.addData("Heading", autoParking.getHeading());
        Monitor.addData("IMU Heading", imu.getHeading());
        Monitor.addData("Webcam Heading", webcam.getHeading());
        Monitor.addData("Position", autoParking.getPosition());
        Monitor.addData("Image detected", webcam.isImageDetected());
        Monitor.addData("Distance", autoParking.getDistance());
        Monitor.addData("Direction error", autoParking.getDirectionError());
        
        super.loop();
    }
    
    TimeStep dropServoReturn = new TimeStep() {
        @Override
        protected void func(double val) {
            dropServo.setPosition(DROP_SERVO_POSITION1);
            dropState = false;
            Monitor.log("Drop servo back in position");
        }
    };
    
    boolean ballDeployEnabled = false;
    int ballDeployStep = 0;
    double ballDeployTime = 0;
    
    void ballDeployStart() {
        if(ballDeployEnabled == true)
            return;
        
        ballDeployStop();
        dropServoReturn.clear();
        belt2.setPower(0);
        ballDeployEnabled = true;
        ballDeployTime = time;
        
        // Step 1: Drop the ball from the front top container
        ballDeployStep = 1;
        dropServo.setPosition(DROP_SERVO_POSITION2);
        dropServo.setNextPosition(DROP_SERVO_POSITION1, 1.1);
    }
    
    void ballDeployLoop() {
        if(ballDeployEnabled == false)
            return;
        
        switch(ballDeployStep) {
          case 1:
            if(time - ballDeployTime >= 1.8) {
                // Step 2: Load 2nd ball into the top front from the belt
                ballDeployStep = 2;
                pushServo.setPosition(PUSH_SERVO_POSITION2);
                pushServo.setNextPosition(PUSH_SERVO_POSITION1, 0.7);
            }
            break;
          case 2:
            if(time - ballDeployTime >= 3.2) {
                // Step 3: Move 3rd ball to the top. Drop 2nd ball.
                ballDeployStep = 3;
                belt2State = true;
                belt2.setPower(BELT2_POWER);
                dropServo.setPosition(DROP_SERVO_POSITION2);
                dropServo.setNextPosition(DROP_SERVO_POSITION1, 1.1);
            }
            break;
          case 3:
            if(time - ballDeployTime >= 6) {
                // Step 4: Load the last ball into the top front from the belt
                ballDeployStep = 4;
                belt2State = false;
                belt2.setPower(0);
                pushServo.setPosition(PUSH_SERVO_POSITION2);
                pushServo.setNextPosition(PUSH_SERVO_POSITION1, 0.7);
            }
            break;
          case 4:
            if(time - ballDeployTime >= 6.9) {
                // Step 5: Drop the last ball.
                ballDeployStep = 5;
                dropServo.setPosition(DROP_SERVO_POSITION2);
                dropServo.setNextPosition(DROP_SERVO_POSITION1, 1.1);
            }
            break;
          case 5:
            if(time - ballDeployTime >= 8.7)
                ballDeployStop();
            break;
        }
    }
    
    void ballDeployStop() {
        if(ballDeployEnabled == true) {
            ballDeployEnabled = false;
            ballDeployStep = 0;
            belt2.setPower(0);
            Monitor.log("Quick deploy finished");
        }
    }
    
    public void stop() {
        extensionServo1.setPosition(0);
        extensionServo2.setPosition(0);
        super.stop();
    }
}
