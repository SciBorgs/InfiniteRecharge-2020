package frc.robot.helpers;

import java.io.Serializable;

public class Point implements Serializable {
    private static final long serialVersionUID = 2;

    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object o) {
        if (o.getClass() != Point.class) {return false;}
        Point point = (Point) o;

        return this.x - point.x <= Geo.getEpsilon()
            && this.y - point.y <= Geo.getEpsilon();
    }

    @Override
    public String toString() {
        return getClass().getName() + "(" + x + "," + y + ")";
    }
}
