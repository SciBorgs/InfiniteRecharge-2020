package frc.robot.helpers;

import frc.robot.helpers.Point;

public abstract class LineLike{

    public Point p1, p2;

    public Line toLine(){
        return new Line(p1, p2);
    }
    public abstract boolean contains(Point p);
    public abstract Point[] getBounds();
}