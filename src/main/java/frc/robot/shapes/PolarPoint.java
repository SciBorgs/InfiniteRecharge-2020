package frc.robot.shapes;

import frc.robot.Utils;

public class PolarPoint {
    public double r, theta;

    public PolarPoint(double r, double theta) {
        this.r     = r;
        this.theta = theta;
    }

    public boolean equals(Object o) {
        if (o.getClass() != Point.class) {return false;}
        PolarPoint point = (PolarPoint) o;

        return Utils.impreciseEquals(this.r, point.r)
            && Utils.impreciseEquals(this.theta, point.theta);
    }

    public String toString() {
        return getClass().getName() + "(" + r + "," + theta + ")";
    }
}
