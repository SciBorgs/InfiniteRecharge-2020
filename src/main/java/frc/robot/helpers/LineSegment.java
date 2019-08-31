package frc.robot.helpers;

import frc.robot.Utils;

public class LineSegment extends LineLike implements AlmostLine{
    public Point p1, p2;
    public LineSegment(Point p1, Point p2){
        this.p1 = p1;
        this.p2 = p2;
    }

    public boolean contains(Point p){
        return this.toLine().contains(p) && 
            Utils.inBounds(p.x, new Pair<>(p1.x, p2.x)) && 
            Utils.inBounds(p.y, new Pair<>(p1.y, p2.y));
    }

    public Point[] getBounds(){
        return new Point[]{p1, p2};
    }
}