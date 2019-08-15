package frc.robot.helpers;

public class Vector{

    public  double x, y;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector copy() { return new Vector(x, y); }

    public void set(double x, double y) {
		this.x = x;
		this.y = y;
	}  

    public double magnitude(Vector A){ return Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2)); }

    public static double Dot(Vector A, Vector B) { return A.x * B.x + A.y * B.y; }
	public double Dot(Vector A) { return this.x * A.x + this.y * A.y; }
    
    public static double angleBetween(Vector A, Vector B) {
		double dot = Dot(A, B);
		double magnitudeA = Math.sqrt(Math.pow(A.x, 2) + Math.pow(A.y, 2));
		double magnitudeB = Math.sqrt(Math.pow(B.x, 2) + Math.pow(B.y, 2));
		return Math.acos(dot / (magnitudeA * magnitudeB));
    }
    
    public static double distanceBetween(Vector A, Vector B){
        return Math.sqrt(Math.pow(B.x - A.x, 2) + Math.pow(B.x - A.x, 2));
    }

}