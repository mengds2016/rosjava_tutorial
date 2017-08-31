package com.wmlynar.ros.neato.distancecalibration;

import java.net.URI;

import org.ros.RosCore;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

public class MainSimulatedWithCore {
	private static String[] EMPTY = {""}; 
	private static ScanOdomPublisher publisher = new ScanOdomPublisher();
	private static ScanOdomSubscriberPlotter subscriber = new ScanOdomSubscriberPlotter();

	public static void main(String[] args) {
		
		// start roscore
		RosCore rosCore = RosCore.newPublic(11311);
		rosCore.start();
		try {
	        rosCore.awaitStart();
	    } catch (Exception e) {
	        throw new RuntimeException(e);
	    }
		
		// Set up the executor for both of the nodes
		NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();

	    NodeConfiguration nodeConfiguration=null;
		if(args.length==0) {
			args = EMPTY;
		}
	    CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(args));
		nodeConfiguration = loader.build();
		

		nodeMainExecutor.execute(publisher, nodeConfiguration);
		nodeMainExecutor.execute(subscriber, nodeConfiguration);

	}

}
