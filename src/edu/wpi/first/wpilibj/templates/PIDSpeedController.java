/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Robotics
 */
public class PIDSpeedController implements SpeedController {

    private PIDController pidController;
    private PIDSource lol;
    private SpeedController speedController;
    double val;
    
    public PIDSpeedController (SpeedController sc, PIDSource ps) {
        speedController = sc;
        pidController = new PIDController(.00015, 0, 0, ps, sc);
        pidController.enable();
        pidController.setSetpoint(0);
    }
    
    public double get() {
        return val;
    }

    public void set(double x, byte b) {
        set(x);
    }

    public void set(double x) {
        pidController.setSetpoint(x * 5000);
        val = x;
    }

    public void disable() {
        pidController.disable();
    }

    public void pidWrite(double x) {
        set(x);
    }
    
    public double getMotorSetting() {
        return speedController.get();
    }
}
