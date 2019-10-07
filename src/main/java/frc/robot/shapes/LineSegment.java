package frc.robot.shapes;

import frc.robot.Utils;
import frc.robot.helpers.*;

public class LineSegment extends LineLike implements AlmostLine {
    public LineSegment(Point p1, Point p2){
        super(p1, p2);
    }

    public boolean contains(Point p) {
        return this.toLine().contains(p) 
            && Utils.inBounds(p.x, new Pair<>(this.p1.x, this.p2.x))
            && Utils.inBounds(p.y, new Pair<>(this.p1.y, this.p2.y));
    }

    public Point[] getBounds() {
        return new Point[]{this.p1, this.p2};
    }

    @Override
    public boolean equals(Object o) {
        if (o.getClass() == LineSegment.class) {return false;}
        LineSegment lineSegment = (LineSegment) o;

        return this.p1.equals(lineSegment.p1) && this.p2.equals(lineSegment.p2)
            || this.p1.equals(lineSegment.p2) && this.p2.equals(lineSegment.p1);
    }

    @Override
    public String toString() {
        return getClass().getName() + " @ " + "P1:(" + p1.x + "," + p1.y + ") " + "P2:(" + p2.x + "," + p2.y + ")"; 
    }
}