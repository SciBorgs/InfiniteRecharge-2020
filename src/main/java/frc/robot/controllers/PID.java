package frc.robot.controllers;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utils;
import frc.robot.dataTypes.Deque;

import java.util.ArrayList;

public class PID {  

	Timer timer;
	private Deque<Double> times, errors;
	private int maxSize = 4;
	private double p, i, d, output, integral, negligibleOutput;

	public PID(double p, double i, double d) {
		this.timer = new Timer();
		this.timer.start();
		this.times  = new Deque<Double>(maxSize);
		this.errors = new Deque<Double>(maxSize);
		this.p = p;
		this.i = i;
		this.d = d;
		this.negligibleOutput = 0;
	}
	
	public Deque<Double> getErrors(){return this.errors;}
	public Deque<Double> getTimes() {return this.times;}
	public void reset() {
		this.output   = 0;
		this.integral = 0;
		this.times  = new Deque<Double>(maxSize);
		this.errors = new Deque<Double>(maxSize);
	}
	
	public void setP(double p) {this.p = p;}
	public void setI(double i) {this.i = i;}
	public void setD(double d) {this.d = d;}
	
	public double getP() {return this.p;}
	public double getI() {return this.i;}
	public double getD() {return this.d;}
	
	public void setSmoother(int maxSize) {
		this.maxSize = maxSize;
		this.times.maxLength  = this.maxSize;
		this.errors.maxLength = this.maxSize;
	}

	private double dError(double error){
		// Get's change in error
		if (this.errors.isEmpty()) {
			return 0;
		} else {
			return error - this.errors.last(); 
		}
	}
	private double dt(boolean smoothed){
		if (this.errors.isEmpty()){
			return 1;
		} else {
			return this.timer.get() - (smoothed ? this.times.last() : this.times.first());
		}
	}
	  
	public void addMeasurement(double error) {
		addMeasurementWithDerivative(error, dError(error) / dt(true));
	}
	  
	public void addMeasurementWithDerivative(double error, double derivative) {
		// This is split up into two functions because often we will have a better estimate of the derivative of the error
		// The split up allows us to use that better estimate by calling this step directly
		double currentTime = timer.get();
		this.integral *= Math.pow(.7, dt(false));
		this.integral += error * dt(false);
		this.output = this.p * error + this.d * derivative + this.i * this.integral;
		this.times.add(currentTime);
		this.errors.add(error);
	}
	  
	public double getOutput() {return this.output;}
	public double getLimitedOutput(double limit) {return Utils.limitOutput(this.output,limit);}
	public void setNegligibleOutput(double negligibleOutput){this.negligibleOutput = negligibleOutput;}
	public boolean isOutputNegligible(){
		return Math.abs(getOutput()) < this.negligibleOutput;
	}
}