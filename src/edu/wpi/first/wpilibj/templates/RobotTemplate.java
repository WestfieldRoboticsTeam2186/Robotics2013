/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be  modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.AnalogPotentiometer;
import java.lang.Math;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    Joystick plane, ps3;
    Talon launcher;
    Victor leftArm, rightArm;
    RobotDrive drive;
    Timer timer;
    Gyro gyro;
    Encoder leftFrontEnc, rightFrontEnc, leftRearEnc, rightRearEnc;
    DriverStationLCD lcd;
    DigitalInput firedSwitch;
    DigitalInput barUpSwitch;
    DigitalInput tautSwitch;
    DigitalInput loaderdownSwitch;
    DigitalInput loaderupSwitch;
    AnalogPotentiometer pot;
    int state;
    PIDSpeedController leftFront, rightFront, leftRear, rightRear;
   
    /*private final static int STATE_DEFAULT = 0;
    private final static int STATE_WINDING = 1;
    private final static int STATE_UNWINDING = 4;
    private final static int STATE_UNWOUND = 5;
    private final static int STATE_FIRING = 6;*/
    
    private final static int WINDING = 0;
    private final static int UNWINDING = 1;
    private final static int SHOOTING = 2;
    private final static int WAITING = 3;
    private final static int AUTO_REWIND = 4;
    private final static int AUTO_DRIVE = 5;
    
    private final static double LOADER_UP = .35;
    private final static double LOADER_DOWN = .2;
    
    private boolean autoFired = false; 
    
    private final static int LAUNCHER_WIND = 1;
    private final static int LAUNCHER_UNWIND = -1;
    
    private final static double INCHES_PER_REV = 3 * 2 * Math.PI;
    private final static double DRIVE_GEAR_TEETH = 15;
    private final static double WHEEL_GEAR_TEETH = 22;
    private final static double TICKS_PER_REV = 1440;
    private final static double AUTO_DRIVE_INCHES = 6 * 12; // in inches
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        plane = new Joystick(1);
        ps3 = new Joystick(2);
        leftFrontEnc = new Encoder(1, 2);
        leftFrontEnc.start();
        leftFrontEnc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
        rightFrontEnc = new Encoder(3,4);
        rightFrontEnc.start();
        rightFrontEnc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
        leftRearEnc = new Encoder(5, 6);
        leftRearEnc.start();
        leftRearEnc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
        rightRearEnc = new Encoder(7,8);
        rightRearEnc.start();
        rightRearEnc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
        leftFront = new PIDSpeedController(new CompensatedSpeedController(new Talon(1), .025, .9), leftFrontEnc);
        rightFront = new PIDSpeedController(new CompensatedSpeedController(new Talon(2), .015, 1.5), rightFrontEnc);
        leftRear = new PIDSpeedController(new CompensatedSpeedController(new Talon(3), .025, 1.15), leftRearEnc);
        rightRear = new PIDSpeedController(new CompensatedSpeedController(new Talon(4), .025, 1.1), rightRearEnc);
        launcher = new Talon(5);
        leftArm = new Victor(6);
        rightArm = new Victor(7);
        drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
        // FIXME: invert left/right motors?
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        //gyro = new Gyro(1);
        //gyro.setSensitivity(0.007);
        firedSwitch = new DigitalInput(14);
        loaderdownSwitch = new DigitalInput(10);
        loaderupSwitch = new DigitalInput(11);
        barUpSwitch = new DigitalInput(12);
        tautSwitch = new DigitalInput(13);
        pot = new AnalogPotentiometer(2);
        lcd = DriverStationLCD.getInstance();
        state = WINDING;
    }
    
    public void autonomousInit() {
        state = WINDING;
        timer = new Timer();
        timer.start();
        drive.arcadeDrive(0,0);
        leftFrontEnc.reset();
        rightFrontEnc.reset();
        leftRearEnc.reset();
        rightRearEnc.reset();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        switch(state) {
            case AUTO_DRIVE:
                double inches = leftFrontEnc.getRaw() / TICKS_PER_REV / DRIVE_GEAR_TEETH *
                        WHEEL_GEAR_TEETH * INCHES_PER_REV;
                if (inches < -AUTO_DRIVE_INCHES){
                    state = UNWINDING;
                }
                break;
            case WINDING:
                if (!barUpSwitch.get()) {
                    state = AUTO_DRIVE;
                }
                break;
                
            case UNWINDING:
                if (barUpSwitch.get()){
                    state = WAITING;
                }else if (tautSwitch.get()){
                    state = SHOOTING;
                    timer.reset();
                }
                break;
                
            case SHOOTING:
                if (!tautSwitch.get()){
                    state = WAITING;
                }else if(barUpSwitch.get()){
                    state = WAITING;
                }else if(!firedSwitch.get()){
                    state = AUTO_REWIND;
                }
                break;
                
            case WAITING:
                if (!ps3.getRawButton(2) && !ps3.getRawButton(6) && ps3.getRawButton(8)) {
                    state = WINDING;
                }
            break;
            case AUTO_REWIND:
                if (!barUpSwitch.get()) {
                    state = WAITING;
                }
        }
        
        switch(state){
            case AUTO_DRIVE:
                launcher.set(0);
                drive.mecanumDrive_Cartesian(0, -.4, 0, 0);
            break;
            case WINDING:
                drive.mecanumDrive_Cartesian(0, 0, 0, 0);
                launcher.set(LAUNCHER_WIND);
            break;
            case AUTO_REWIND:
                launcher.set(LAUNCHER_WIND);
            break;
            case UNWINDING:
                drive.mecanumDrive_Cartesian(0, 0, 0, 0);
                launcher.set(LAUNCHER_UNWIND);
                if (loaderdownSwitch.get()){
                    leftArm.set(-LOADER_DOWN);
                    rightArm.set(LOADER_DOWN);
                }else{
                    leftArm.set(0);
                    rightArm.set(0);
                }    
            break;
            case SHOOTING:
                if (timer.get() > 1.5){
                    launcher.set(LAUNCHER_UNWIND);
                }else{
                    launcher.set(0);
                }
            break;
            case WAITING:
                launcher.set(0);
            break;            
            default:
                launcher.set(0);
            break;    
        }
        if (tautSwitch.get()){
            lcd.println(DriverStationLCD.Line.kUser1, 1,"ready to fire");
        }else{
            lcd.println(DriverStationLCD.Line.kUser1, 1,"DON'T FIRE   ");
        }
        lcd.println(DriverStationLCD.Line.kUser3, 1, "wound switch: " + barUpSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser4, 1, "taut switch: " + tautSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser5, 1, "fired switch: " + firedSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser6, 1, "stateNum: " + state);
        lcd.updateLCD();
                
        /* if (timer.get() < 3) {
            drive.mecanumDrive_Cartesian(0, 1, 0, gyro.getAngle() % 360);
        }else{
            drive.mecanumDrive_Cartesian (0, 0, 0, gyro.getAngle() %360);
            
        }
        if(barUpSwitch.get() && tautSwitch.get() && !autoFired){
            autoFired = true; 
        }else if(!barUpSwitch.get() && !autoFired){
            if (loaderdownSwitch.get()){
                leftArm.set(-LOADER_DOWN);
                rightArm.set(LOADER_DOWN);
            }else{
                leftArm.set(0);
                rightArm.set(0);
            }
        }
        
        if(!barUpSwitch.get() &&!autoFired){
             launcher.set(LAUNCHER_UNWIND);
        }else if(barUpSwitch.get()){
            launcher.set(LAUNCHER_WIND);
        } else {
            launcher.set(0);
        } */
    }
    public void teleopInit() {
        if (state == AUTO_REWIND){
            state = WINDING;
        }
        if (state == AUTO_DRIVE){
            state = WINDING; 
        }
    }
    
//    public void mecanum(){
//        double x = plane.getX();
//        double y = plane.getY();
//        double angle = Math.(y,x) + Math.PI/4.0;
//        double mag = Math.sqrt(x*x+y*y);
//        double cosD = Math.cos(angle);
//        double sinD = Math.sin(angle);
//        
//        
//        
//    }
    /**  * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        drive.mecanumDrive_Cartesian(plane.getX(), plane.getY(), plane.getZ(), 0);
        //Control the arm
        if (ps3.getRawButton(5) && !ps3.getRawButton(7) && loaderupSwitch.get()){
            leftArm.set(LOADER_UP);
            rightArm.set(-LOADER_UP); 
        }else if (ps3.getRawButton(7)&& !ps3.getRawButton(5) && loaderdownSwitch.get()){
            leftArm.set(-LOADER_DOWN);
            rightArm.set(LOADER_DOWN);
        }else{
            leftArm.set(0);
            rightArm.set(0);
        }
        //Control the launcher
  /*      if(barUpSwitch.get() && ps3.getRawButton(8)){
            state = WINDING;
        }else if(!barUpSwitch.get() && ps3.getRawButton(8) && !tautSwitch.get()){
            state = UNWINDING;
        }else if(!barUpSwitch.get() && ps3.getRawButton(2) && ps3.getRawButton(6) && tautSwitch.get()){
            state = SHOOTING;
        }else{
            state = WAITING;
        } */
        
        
        switch(state) {
            case WINDING:
                if (!barUpSwitch.get()) {
                    state = UNWINDING;
                }
                break;
                
            case UNWINDING:
                if (barUpSwitch.get()){
                    state = WAITING;
                }else if (tautSwitch.get()){
                    state = SHOOTING;
                }
                break;
                
            case SHOOTING:
                if (!tautSwitch.get()){
                    state = WAITING;
                }else if(barUpSwitch.get()){
                    state = WAITING;
                }else if(!firedSwitch.get()){
                    state = WAITING;
                }
                break;
                
            case WAITING:
                if (!ps3.getRawButton(2) && !ps3.getRawButton(6) && !ps3.getRawButton(8)) {
                    state = WINDING;
                }
        }
        switch(state){
            case WINDING:
                if(ps3.getRawButton(8)){
                    launcher.set(LAUNCHER_WIND);
                }else{
                    launcher.set(0);
                }
            break;
            case UNWINDING:
                if(ps3.getRawButton(8)){
                    launcher.set(LAUNCHER_UNWIND);
                }else{
                    launcher.set(0);
                }
            break;
            case SHOOTING:
                if(ps3.getRawButton(2) && ps3.getRawButton(6)){
                    launcher.set(LAUNCHER_UNWIND);
                }else{
                    launcher.set(0);
                }
            break;
            default:
                launcher.set(0);
            break;    
                
        }           
        
        // janky direct motor control launcher code
        /*if(ps3.getRawButton(6)) {
            launcher.set(1);
        } else if (ps3.getRawButton(8)) {
            launcher.set(-1);
        } else {
            launcher.set(0);
        }*/
        
        //test.set(plane.getRawAxis(1));
        /*lcd.println(DriverStationLCD.Line.kUser1, 1, "LF Encoder: " + leftFrontEnc.getRate());
        lcd.println(DriverStationLCD.Line.kUser2, 1, "RF Encoder: " + rightFrontEnc.getRate());
        lcd.println(DriverStationLCD.Line.kUser3, 1, "LR Encoder: " + leftRearEnc.getRate());
        lcd.println(DriverStationLCD.Line.kUser4, 1, "RR Encoder: " + rightFrontEnc.getRate());*/
        if (tautSwitch.get()){
            lcd.println(DriverStationLCD.Line.kUser1, 1,"ready to fire");
        }else{
            lcd.println(DriverStationLCD.Line.kUser1, 1,"DON'T FIRE   ");
        }
        lcd.println(DriverStationLCD.Line.kUser2, 1, "loader up switch: " + loaderupSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser3, 1, "wound switch: " + barUpSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser4, 1, "taut switch: " + tautSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser5, 1, "fired switch: " + firedSwitch.get());
        lcd.println(DriverStationLCD.Line.kUser6, 1, "stateNum: " + state);
        lcd.updateLCD();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
         
    }
    
}
