package frc.robot.shapes;

public interface AlmostLine{

    public Line toLine();
    public boolean contains(Point p);
    public Point[] getBounds();

}