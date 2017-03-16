package org.usfirst.frc.team2635.robot.model;

import java.text.SimpleDateFormat;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

public class Vision {
	CvSink cvSink;
	Mat source;
	GripPipeline GripPipeline;
	CvSource cvSource;
	ArrayList<Rect> boundRect;
	ArrayList<MatOfPoint> grip;
	UsbCamera camera;
	String currentdatehour;
	String currentdate;
	public Vision(UsbCamera camera) {
		this.camera = camera;
	}
	
	public Vision() {
	}
	
	
	public void camInit(){
		GripPipeline = new GripPipeline();
		
		camera.setResolution(640, 480);
		cvSink = new CvSink("cvSink");
		cvSink.setSource(camera);
		source = new Mat();
		cvSource = CameraServer.getInstance().putVideo("new", 640, 480);
		
		currentdate = new SimpleDateFormat("MM/dd/yyy").format(new java.util.Date());
		
	}
	
	public void createBox(){
		boundRect = new ArrayList<Rect>();
		//Get frame from camera
		
		
		
		cvSink.grabFrame(source);
		
		//Use Grip Code
		GripPipeline.process(source);
		grip = GripPipeline.findContoursOutput();
		//set and draw all boxes
		for( int i = 0; i< grip.size(); i++ ){
			Imgproc.drawContours(source, grip, i, new Scalar(255, 0,0),1);
			boundRect.add(Imgproc.boundingRect(grip.get(i)));
			//Uncomment to view all drawn boxes
			//Rect rect = Imgproc.boundingRect(grip.get(i));
			//Imgproc.rectangle( source, rect.tl(), rect.br(), new Scalar(0,0,255), 2, 8, 0 );
		    }
		
		
	}
}
