package frc.robot.colors;

import java.lang.IllegalArgumentException;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * RGB
 * 
 * <p>Values are between 0 and 1, inclusive.</p>
 */
public class RGB {
    private double r, g, b;
    public RGB(double red, double green, double blue) {
        if (red < 0 || red > 1) {
            throw new IllegalArgumentException("Red outside of range");
        }
        if (green < 0 || green > 1) {
            throw new IllegalArgumentException("Green outside of range");
        }
        if (blue < 0 || blue > 1) {
            throw new IllegalArgumentException("Blue outside of range");
        }
        this.r = red;
        this.g = green;
        this.b = blue;
    }

    public Color toColor() {
        return new Color(toColor8Bit());
    }
    public Color8Bit toColor8Bit() {
        return new Color8Bit((int)(r*255.0), (int)(g*255.0), (int)(b*255.0));
    }

    public double getR() {
        return r;
    }

    public void setR(double r) {
        this.r = r;
    }

    public double getG() {
        return g;
    }

    public void setG(double g) {
        this.g = g;
    }

    public double getB() {
        return b;
    }

    public void setB(double b) {
        this.b = b;
    }
    public boolean equals(RGB other) {
        return this.r == other.r && this.g == other.g && this.b == other.b;
    }
}