package frc.robot.pathFollowing;

public class DriveBase {
	private double left;
    private double right;
    
	public DriveBase(double left, double right) {
		this.left = left;
		this.right = right;
	}

	public double getLeft()  { return left;  }
	public double getRight() { return right; }
}