/*
 * Copyright (C) 2014 woj.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.wmlynar.ros.neato.calibration;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import org.jfree.ui.RefineryUtilities;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.wmlynar.ros.neato.utils.XyTimePlotter;

import nav_msgs.Odometry;
import sensor_msgs.LaserScan;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class ScanOdomSubscriberPlotter extends AbstractNodeMain {

	private Subscriber<nav_msgs.Odometry> odomSubscriber;
	private Subscriber<sensor_msgs.LaserScan> scanSubscriber;
	private XyTimePlotter plotter;
	
	private double bias1 = 0;
	private boolean isBias1Set = false;
	private double bias2 = 0;
	private boolean isBias2Set = false;
	
	private double distance = 0;
	private double prevX;
	private double prevY;
	private boolean isPrevSet = false;
	
	private boolean paused = false;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("neato/scanOdomPlotter");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		
		plotter = new XyTimePlotter("Scan and Odom");
		RefineryUtilities.centerFrameOnScreen(plotter);
		plotter.setVisible(true);
		
		plotter.setMaximumXRange(10);
		
		plotter.addKeyListener(new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent e) {
				pauseUnpause();
			}
			
			@Override
			public void keyReleased(KeyEvent e) {
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
			}
		});
		
		odomSubscriber = connectedNode.newSubscriber("odom", nav_msgs.Odometry._TYPE);
		odomSubscriber.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
			@Override
			public void onNewMessage(nav_msgs.Odometry message) {
				try {
					onOdomMessage(message);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}

		});

		scanSubscriber = connectedNode.newSubscriber("scan", sensor_msgs.LaserScan._TYPE);
		scanSubscriber.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
			@Override
			public void onNewMessage(sensor_msgs.LaserScan message) {
				try {
					onScanMessage(message);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}
	
	public void pauseUnpause() {
		if(!paused) {
			paused = true;
		} else {
			// reset bias
			isBias1Set = false;
			isBias2Set = false;
			paused = false;
		}
	}
	
	private void onOdomMessage(Odometry message) {
		double timestamp = message.getHeader().getStamp().toSeconds();
		
		double valueX = message.getPose().getPose().getPosition().getX();
		double valueY = message.getPose().getPose().getPosition().getY();
		
		if(!isPrevSet) {
			prevX = valueX;
			prevY = valueY;
			
			isPrevSet = true;
			return;
		}

		double dx = valueX-prevX;
		double dy = valueY-prevY;
		
		double value = Math.sqrt(dx*dx+dy*dy);

		distance += value;
		
		prevX = valueX;
		prevY = valueY;
		
		if(!isBias1Set) {
			//bias1 = distance;
			distance = 0;
			isBias1Set = true;
		}
		//distance -= bias1;
		if(!paused) {
			plotter.addValues("odom",timestamp,distance);
			Utils.logCsv("odom",timestamp,distance);
		}
	}
	
	private void onScanMessage(LaserScan message) {
		double timestamp = message.getHeader().getStamp().toSeconds();
		
		float[] ranges = message.getRanges();
		float minRange = message.getRangeMin();//-10000000000000.f;
		float maxRange = message.getRangeMax();//100000000000000000.f;
		float value = Utils.averageInBounds(Utils.getArray(ranges,178,182), minRange, maxRange, -1);
		if(!isBias2Set) {
			bias2 = value;
			isBias2Set = true;
		}
		value-=bias2;
		if(!paused) {
			plotter.addValues("scan",timestamp,value);
			Utils.logCsv("scan",timestamp,value);
		}
	}

}

