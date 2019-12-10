package frc.robot.controllers;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils;
import java.util.ArrayList;

public class PID {  

	Timer timer;
	private ArrayList<Double> times, errors;
	private int maxSize = 4;
	private double p, i, d, output, integral, negligibleOutput;

	public PID(double p, double i, double d) {
		this.timer = new Timer();
		this.timer.start();
		this.times  = new ArrayList<Double>();
		this.errors = new ArrayList<Double>();
		this.p = p;
		this.i = i;
		this.d = d;
		this.negligibleOutput = 0;
	}
	
	public ArrayList<Double> getErrors(){return this.errors;}
	public ArrayList<Double> getTimes() {return this.times;}
	public void reset() {
		this.output   = 0;
		this.integral = 0;
		this.times  = new ArrayList<Double>();
		this.errors = new ArrayList<Double>();
	}
	
	public void setP(double p) {this.p = p;}
	public void setI(double i) {this.i = i;}
	public void setD(double d) {this.d = d;}
	
	public double getP() {return this.p;}
	public double getI() {return this.i;}
	public double getD() {return this.d;}
	
	public void setSmoother(int maxSize) {this.maxSize = maxSize;}

	private double dError(double error){
		// Get's change in error
		if (this.errors.isEmpty()) {
			return 0;
		} else {
			// errors.get(0) will be the farthest back error b/c trimadd adds to the end of the arraylist
			return error - this.errors.get(0); 
		}
	}
	private double dt(){
		if (this.errors.isEmpty()){
			return 1;
		} else {
			return this.timer.get() - this.times.get(0);
		}
	}
	  
	public void addMeasurement(double error) {
		addMeasurementWithDerivative(error, dError(error) / dt());
	}
	  
	public void addMeasurementWithDerivative(double error, double derivative) {
		// This is split up into two functions because often we will have a better estimate of the derivative of the error
		// The split up allows us to use that better estimate by calling this step directly
		double currentTime = timer.get();
		this.integral += error * dt();
		this.output = this.p * error + this.d * derivative + this.i * this.integral;
		Utils.trimAdd(this.times, currentTime, this.maxSize);
		Utils.trimAdd(this.errors, error, this.maxSize);
	}
	  
	public double getOutput() {return this.output;}
	public double getLimitedOutput(double limit) {return Utils.limitOutput(this.output,limit);}
	public void setNegligibleOutput(double negligibleOutput){this.negligibleOutput = negligibleOutput;}
	public boolean outputIsNegligible(){
		return Math.abs(getOutput()) < this.negligibleOutput;
	}
}