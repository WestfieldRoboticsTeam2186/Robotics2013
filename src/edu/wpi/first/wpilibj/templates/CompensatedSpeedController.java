/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Robotics
 */
public class CompensatedSpeedController implements SpeedController {
    
    private SpeedController inner;
    private double add;
    private double scale;
    private double externalVal;
    
    public CompensatedSpeedController(SpeedController sc, double add_, double scale_) {
        inner = sc;
        add = add_;
        scale = scale_;
    }

    public double get() {
        return externalVal;
    }

    public void set(double d, byte b) {
        externalVal = d;
        inner.set((d + add) * scale, b);
    }

    public void set(double d) {
        externalVal = d;
        inner.set((d + add) * scale);
    }

    public void disable() {
        inner.disable();
    }

    public void pidWrite(double d) {
        externalVal = d;
        inner.pidWrite((d + add) * scale);
    }
    
}
