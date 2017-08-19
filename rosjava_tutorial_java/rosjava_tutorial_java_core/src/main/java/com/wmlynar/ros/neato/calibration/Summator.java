package com.wmlynar.ros.neato.calibration;

public class Summator {
	
	private double value = 0;
	
	public void add(double value) {
		this.value += value;
	}

	public double getValue() {
		return value;
	}

}
