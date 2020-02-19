package frc.robot.colors;

import java.lang.IllegalArgumentException;

/**
 * CMYK
 * 
 * <p>Values are between 0 and 1, inclusive.</p>
 */
public class CMYK {
    private double c, m, y, k;
    public CMYK(double cyan, double magenta, double yellow, double black) {
        if (cyan < 0 || cyan > 1) {
            throw new IllegalArgumentException("Cyan outside of range");
        }
        if (magenta < 0 || magenta > 1) {
            throw new IllegalArgumentException("Magenta outside of range");
        }
        if (yellow < 0 || yellow > 1) {
            throw new IllegalArgumentException("Yellow outside of range");
        }
        if (black < 0 || black > 1) {
            throw new IllegalArgumentException("Black outside of range");
        }
        this.c = cyan;
        this.m = magenta;
        this.y = yellow;
        this.k = black;
    }

    public double getC() {
        return c;
    }

    public void setC(double c) {
        this.c = c;
    }

    public double getM() {
        return m;
    }

    public void setM(double m) {
        this.m = m;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getK() {
        return k;
    }

    public void setK(double k) {
        this.k = k;
    }
}