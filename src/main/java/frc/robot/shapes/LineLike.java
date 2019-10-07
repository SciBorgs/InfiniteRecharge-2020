package frc.robot.shapes;

public abstract class LineLike{

    public Point p1, p2;

    protected LineLike(Point p1, Point p2){
        this.p1 = p1;
        this.p2 = p2;
    }

    public Line toLine(){
        return new Line(p1, p2);
    }
    public abstract boolean contains(Point p);
    public abstract Point[] getBounds();
}