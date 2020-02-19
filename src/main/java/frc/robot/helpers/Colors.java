package frc.robot.helpers;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Utils;
import frc.robot.colors.*;
/**
 * Colors
 */
public class Colors {

    public static HSV toHSV(RGB rgb) {
        double cmax = Utils.max(rgb.getR(), rgb.getG(), rgb.getB());
        double delta = cmax - Utils.min(rgb.getR(), rgb.getG(), rgb.getB());
        double hue;
        if (delta == 0) {
            hue = 0;
        } else if (cmax == rgb.getR()) {
            hue = Utils.mod((rgb.getG() - rgb.getB())/delta, 6)/6;
        } else if (cmax == rgb.getG()) {
            hue = (((rgb.getB() - rgb.getR())/delta) + 2)/6;
        } else { // cmax is B
            hue = (((rgb.getR() - rgb.getG())/delta) + 4)/6;
        }
        double saturation = cmax == 0 ? 0 : delta / cmax;
        double value = cmax;
        return new HSV(hue, saturation, value);
    }

    public static HSV toHSV(CMYK cmyk) {
        return toHSV(toRGB(cmyk));
    }

    public static HSV toHSV(Color color) {
        return toHSV(toRGB(color));
    }

    public static HSV toHSV(Color8Bit color) {
        return toHSV(toRGB(color));
    }

    public static RGB toRGB(HSV hsv) {
        double c = hsv.getV() * hsv.getS();
        double x = c * (1 - Math.abs((6 * hsv.getH()) % 2 - 1));
        double m = hsv.getV() - c;
        double r = 0, g = 0, b = 0;
        if (hsv.getH() < 60)         { r = c; g = x;
        } else if (hsv.getH() < 120) { r = x; g = c;
        } else if (hsv.getH() < 180) { g = c; b = x;
        } else if (hsv.getH() < 240) { g = x; b = c;
        } else if (hsv.getH() < 300) { r = x; b = c;
        } else                       { r = c; b = x; } // 300 <= H < 360 
        return new RGB(r + m, g + m ,b + m);
    }

    public static RGB toRGB(CMYK cmyk) {
        return new RGB((1 - cmyk.getC()) * (1 - cmyk.getK()), (1 - cmyk.getM()) * (1 - cmyk.getK()), (1 - cmyk.getY()) * (1 - cmyk.getK()));
    }

    public static RGB toRGB(Color color) {
        return new RGB(color.red, color.blue, color.green);
    }

    public static RGB toRGB(Color8Bit color) {
        return new RGB(color.red/255.0, color.blue/255.0, color.green/255.0);
    }

    public static CMYK toCMYK(RGB rgb) {
        double black = 1 - Utils.max(rgb.getR(), rgb.getG(), rgb.getB());
        return new CMYK((1 - rgb.getR() - black)/(1 - black), (1 - rgb.getG() - black)/(1 - black), (1 - rgb.getB() - black)/(1 - black), black);
    }

    public static CMYK toCMYK(HSV hsv) {
        return toCMYK(toRGB(hsv));
    }

    public static CMYK toCMYK(Color color) {
        return toCMYK(toRGB(color));
    }

    public static CMYK toCMYK(Color8Bit color) {
        return toCMYK(toRGB(color));
    }

    public Color toColor(RGB rgb) {
        return new Color(toColor8Bit(rgb));
    }
    public Color toColor(HSV hsv) {
        return toColor(toRGB(hsv));
    }
    public Color toColor(CMYK cmyk) {
        return toColor(toRGB(cmyk));
    }

    public Color8Bit toColor8Bit(RGB rgb) {
        return new Color8Bit((int)(rgb.getR()*255.0), (int)(rgb.getG()*255.0), (int)(rgb.getG()*255.0));
    }
    public Color8Bit toColor8Bit(HSV hsv) {
        return toColor8Bit(toRGB(hsv));
    }
    public Color8Bit toColor8Bit(CMYK cmyk) {
        return toColor8Bit(toRGB(cmyk));
    }
}