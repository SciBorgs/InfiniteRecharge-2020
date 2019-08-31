package frc.robot.helpers;

public class Ray extends LineLike implements AlmostLine{
    public Point p1, p2; // p1 is the bounds, p2 determines the direction

    public Ray(Point p1, Point p2){
        this.p1 = p1;
        this.p2 = p2;
    }

    public boolean contains(Point p){
        Point diff1 = Geo.subtractPoints(p,  p1);
        Point diff2 = Geo.subtractPoints(p2, p1);
        boolean correctSide = diff1.y * diff2.y > 0 || diff1.x * diff2.x > 0;
        return this.toLine().contains(p) && correctSide;
    }

    public Point[] getBounds(){
        return new Point[]{p1};
    }
}