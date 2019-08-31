package frc.robot.helpers;

import frc.robot.Utils;

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
}