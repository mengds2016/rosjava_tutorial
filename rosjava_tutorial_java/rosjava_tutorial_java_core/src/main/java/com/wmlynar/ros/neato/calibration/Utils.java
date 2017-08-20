package com.wmlynar.ros.neato.calibration;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.HashSet;

import javax.management.RuntimeErrorException;

import com.google.common.io.Files;

public class Utils {

	private static HashSet<String> writed = new HashSet<>();

	public static float averageInBounds(float[] array, float minValue, float maxValue, float otherwise) {
		int num = 0;
		float sum = 0;
		for (int i = 0; i < array.length; i++) {
			float value = array[i];
			if (value >= minValue && value <= maxValue) {
				sum += value;
				num++;
			}
		}
		if (num <= 0) {
			return otherwise;
		}
		return sum / num;
	}

	public static float[] getArray(float[] ranges, int minRange, int maxRange) {
		float[] array = new float[maxRange - minRange + 1];
		for (int i = minRange; i <= maxRange; i++) {
			array[i - minRange] = ranges[i];
		}
		return array;
	}

	public static void logCsv(String filename, double... values) {

		try
		{
			if(!writed.contains(filename)) {
				File file = new File(filename);
				file.mkdirs();
				file.delete();
				writed.add(filename);
			}
			
		    FileWriter writer = new FileWriter(filename,true); //the true will append the new data
			for (int i = 0; i < values.length; i++) {
				String ss = String.format("%f",values[i]);
				writer.append(ss);
				if(i!=values.length-1) {
					writer.append(';');
				}
			}
			writer.write("\n");//appends the string to the file
			writer.close();
		}
		catch(IOException e)
		{
			throw new RuntimeException(e);
		}
	}

}
