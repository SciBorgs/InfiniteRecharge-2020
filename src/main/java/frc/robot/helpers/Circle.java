package frc.robot.helpers;

public class Circle {

    Point p0, p1;
    double x0, y0, x1, y1;
    double k, h , r, m;

    public Circle(Point currPos, double currHeading, Point finalPos) {
        this.p0 = currPos;
        this.p1 = finalPos;
        setVars();
        this.m = Math.atan(currHeading);
        this.h = calculateH();
        this.k = calculateK();
        this.r = calculateR();
    }

    private void setVars() {
        this.x0 = this.p0.x;
        this.y0 = this.p0.y;
        this.x1 = this.p1.x;
        this.y1 = this.p1.y;
    }

    public double getH() { return this.h; }
    public double getK() { return this.k; }
    public double getR() { return this.r; }

    private double calculateH(){
        double a = (.5 * this.y1 * this.y1 - this.y0 * this.y0 + this.x1 * this.x1 - this.x0 * this.x0);
        double b = this.m * this.y0 * this.x1 - this.m * this.y0 * this.x0 + this.x0 * this.x1 - this.x0 * this.x0;
        double c = this.m * this.x0 - this.m * this.x1 - this.y0 + this.y1;
        return (a - b) / c;
    }
    
    private double calculateK() { return this.m * (this.y0 - this.h) + this.x0; }
    private double calculateR() { return Math.sqrt(Math.pow(this.x0 - this.k, 2) + Math.pow(this.y0 - this.h, 2)); }
}