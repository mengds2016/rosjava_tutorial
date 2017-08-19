package com.wmlynar.ros.neato.calibration;

import java.net.URI;

import org.ros.RosCore;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

public class MainSubscriber {
	private static String[] EMPTY = {""}; 
	private static ScanOdomPublisher publisher = new ScanOdomPublisher();
	private static ScanOdomSubscriberPlotter subscriber = new ScanOdomSubscriberPlotter();

	public static void main(String[] args) {
		
		URI masteruri = URI.create("http://127.0.0.1:11311");
		String host = "127.0.0.1";

		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, masteruri);
		NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		
		nodeMainExecutor.execute(subscriber, nodeConfiguration);

	}

}
