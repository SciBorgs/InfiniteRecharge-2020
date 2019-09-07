package frc.robot.helpers;

import frc.robot.helpers.Point;
import frc.robot.helpers.Line;

public interface AlmostLine{

    public Line toLine();
    public boolean contains(Point p);
    public Point[] getBounds();

}