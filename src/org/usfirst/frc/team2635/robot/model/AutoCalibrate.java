package org.usfirst.frc.team2635.robot.model;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;





public class AutoCalibrate extends Vision{
	final static int RectLeft_tl_x = 278;
	final static int RectLeft_tl_y = 305;
	final static int RectLeft_br_x = 299;
	final static int RectLeft_br_y = 356;
	final static int RectRight_tl_x = 363;
	final static int RectRight_tl_y = 305;
	final static int RectRight_br_x = 384;
	final static int RectRight_br_y = 357;
	
	Boolean  replacethis = null;
	
	double[] hsvThresholdHueSolved;
	double[] hsvThresholdSaturationSolved;
	double[] hsvThresholdValueSolved;
	
	double[] hsvThresholdHueStart;
	double[] hsvThresholdSaturationStart;
	double[] hsvThresholdValueStart;
	
	double[] hsvThresholdHueCurrentAttempt;
	double[] hsvThresholdSaturationCurrentAttempt;
	double[] hsvThresholdValueCurrentAttempt;
	
	Rect RectRight;
	Rect RectLeft;
	
	public void CaliInit(){
		Point RectLeft_tl = new Point(RectLeft_tl_x, RectLeft_tl_y);
		Point RectLeft_br = new Point(RectLeft_br_x, RectLeft_br_y);
		Point RectRight_tl = new Point(RectRight_tl_x, RectRight_tl_y);
		Point RectRight_br = new Point(RectRight_br_x, RectRight_br_y);
	
		Imgproc.rectangle( source, RectLeft_tl, RectLeft_br, new Scalar(0,0,255), 2, 8, 0 );
		Imgproc.rectangle( source, RectRight_tl, RectRight_br, new Scalar(0,0,255), 2, 8, 0 );
		
		RectLeft = new Rect(RectLeft_tl, RectLeft_br);
		RectRight = new Rect(RectRight_tl, RectRight_br);
		
		hsvThresholdHueStart        = new double[] {78.0, 107.0};
		hsvThresholdSaturationStart = new double[] {6.0, 58.0};
		hsvThresholdValueStart      = new double[] {244.0, 255.0};
		
	}
	public void Calibrate(){
		if(replacethis){
			cvSource.putFrame(source);
		} else if(!replacethis){
			//button is pressed
			
			//sample inside rectangles
			
			//
		}
	}
}
