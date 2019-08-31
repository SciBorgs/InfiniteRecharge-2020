package frc.robot.helpers;

public class Line extends LineLike{
    public Point p1, p2;

    public Line(Point p1, Point p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

     public boolean contains(Point p){
        return Geo.arePointsCollinear(p1, p2, p);
    }

     public Point[] getBounds(){
        return new Point[0];
    }
}