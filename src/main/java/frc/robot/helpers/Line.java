package frc.robot.helpers;

public class Line extends LineLike{

    public Line(Point p1, Point p2) {
        super(p1, p2);
    }

     public boolean contains(Point p){
        return Geo.arePointsCollinear(this.p1, this.p2, p);
    }

     public Point[] getBounds(){
        return new Point[0];
    }
}