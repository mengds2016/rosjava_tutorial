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

package com.wmlynar.ros.neato.distancecalibration;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import nav_msgs.Odometry;
import sensor_msgs.LaserScan;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class ScanOdomPublisher extends AbstractNodeMain {

	private Publisher<nav_msgs.Odometry> odomPublisher;
	private Publisher<sensor_msgs.LaserScan> scanPublisher;
	private LaserScan scan;
	private Odometry odom;
	private float[] ranges;
	private float xOdom;
	private float xScan;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("neato/fakeOdomScan");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		odomPublisher = connectedNode.newPublisher("odom", nav_msgs.Odometry._TYPE);
		odom = odomPublisher.newMessage();
		
		scanPublisher = connectedNode.newPublisher("base_scan", sensor_msgs.LaserScan._TYPE);
		scan = scanPublisher.newMessage();
		scan.setAngleMin(0);
		scan.setAngleMax((float) (359*Math.PI/180.));
		scan.setAngleIncrement((float) (Math.PI/180.));
		scan.setRangeMin((float) 0.02);
		scan.setRangeMax((float) 0.05);
		ranges = new float[360];
		scan.setRanges(ranges);
		
		xOdom = 252523.f;
		xScan = 7543235.f;

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void loop() throws InterruptedException {
				ScanOdomPublisher.this.loop(connectedNode);
			}
		});
	}
	
	public void loop(ConnectedNode connectedNode) throws InterruptedException {
		
		Time time = connectedNode.getCurrentTime();
		
		odom.getHeader().setStamp(time);
		odom.getPose().getPose().getPosition().setX(xOdom);
		odom.getPose().getPose().getPosition().setY(xOdom);
		odomPublisher.publish(odom);

		scan.getHeader().setStamp(time);
		scan.getRanges()[180] = xScan;
		scanPublisher.publish(scan);
		
		xOdom += Math.random()*10-5;
		xScan += Math.random()*20-10;

		Thread.sleep(10);
	}

}
