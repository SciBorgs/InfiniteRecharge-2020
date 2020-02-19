package frc.robot.colors;

import java.lang.IllegalArgumentException;

/**
 * HSV
 * 
 * <p>Values are between 0 and 1, inclusive, except for hue, which is 0 <= x < 1.</p>
 */
public class HSV {
    private double h, s, v;
    public HSV(double hue, double saturation, double value) {
        if (hue < 0 || hue >= 1) {
            throw new IllegalArgumentException("Hue outside of range: " + hue);
        }
        if (saturation < 0 || saturation > 1) {
            throw new IllegalArgumentException("Saturation outside of range: " + saturation);
        }
        if (value < 0 || value > 1) {
            throw new IllegalArgumentException("Value outside of range: " + value);
        }
        this.h = hue;
        this.s = saturation;
        this.v = value;
    }

    public double getH() {
        return h;
    }

    public void setH(double h) {
        this.h = h;
    }

    public double getS() {
        return s;
    }

    public void setS(double s) {
        this.s = s;
    }

    public double getV() {
        return v;
    }

    public void setV(double v) {
        this.v = v;
    }
}