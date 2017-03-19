package org.usfirst.frc.team2635.robot.model;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterVision extends Vision {
	Rect confRectTop;
	Rect confRectBot;
	Rect confRectFull;
	ArrayList<Rect> reck1;
	ArrayList<Rect> reck2;
	ArrayList<Rect> reck3;
	Integer confirmed;
	Integer welike;
			
	public ShooterVision(UsbCamera camera) {
		super(camera);
	}
	
	@SuppressWarnings("deprecation")
	public void confirmBox() {
		Integer[] poss = new Integer[999];
		
		reck1 = new ArrayList<Rect>();
		reck2 = new ArrayList<Rect>();
		reck3 = new ArrayList<Rect>();
		for (Integer b = 0; b < boundRect.size(); b++) {
			for (Integer j = 1; j < boundRect.size(); j++) {
				confRectTop=null;
				confRectBot=null;
				//Integer j = b;
				if (boundRect.get(b) != null && boundRect.get(j) != null&&b!=j&&j>b) {
					Rect rect1 = boundRect.get(b);
					Rect rect2 = boundRect.get(j);
					Rect temp;
					Integer topH;
					Integer topW;
					Integer botH;
					Integer botW;
					
					//Post height of rectangles for debug
		//			SmartDashboard.putInt("rect1y",rect1.y);
		//			SmartDashboard.putInt("rect2y", rect2.y);
					//Decide which rectangle is top
					Point tl1;
					Point tl2;
					if (rect1.y < rect2.y) {
						topH = rect1.height;
						topW = rect1.width;
						botH = rect2.height;
						botW = rect2.width;
						tl1 = rect1.tl();
						tl2 = rect2.tl();
						temp = Imgproc.boundingRect(new MatOfPoint(rect2.br(),rect1.tl()));
						//Uncomment to see what box is being tested
						//Imgproc.rectangle( source, rect1.tl(), rect1.br(), new Scalar(0,0,255), 2, 8, 0 );
					} else {
						topH = rect2.height;
						topW = rect2.width;
						botH = rect1.height;
						botW = rect1.width;
						tl1 = rect2.tl();
						tl2 = rect2.tl();
						temp = Imgproc.boundingRect(new MatOfPoint(rect1.br(),rect2.tl()));
						//Uncomment to see what box is being tested
						//Imgproc.rectangle( source, rect2.tl(), rect2.br(), new Scalar(0,0,255), 2, 8, 0 );
					}
					//Create variables to be used for confirmation
					Integer topHalfHeight = topH/2;
					Integer totalHeight = (int) (temp.height*0.4);
					
					//Do checks on rectangle pair
					Integer comp1 = topHalfHeight/botH;
					Integer comp2 = totalHeight/topH;
					Integer comp3 = botW/topW;
					Integer comp4 = rect1.x/rect2.x;
					boolean comp5 = (tl1.y-tl2.y)<rect1.width;
					//Post used variables
		//				SmartDashboard.putDouble("topHalfHeight", topHalfHeight);
		//				SmartDashboard.putDouble("botH", botH);
					//Post results of checks
		//				SmartDashboard.putDouble("comp1", comp1);
		//				SmartDashboard.putDouble("comp2", comp2);
		//				SmartDashboard.putDouble("comp3", comp3);
					Imgproc.line(source, new Point(320,0), new Point(320, 480), new Scalar(255,255,255));
					Imgproc.line(source, new Point(0,240), new Point(640, 240), new Scalar(255,255,255));
					Integer done = 1 - (1 * Math.abs(4 - (comp1+comp2+comp3+comp4)));
					if (rect1.width > 15 && rect2.width > 15) {
						for (Integer i=0; i>999; i++) {
							if(poss[i]==null){
								poss[i]=done;
								if (rect1.y < rect2.y) {
									reck1.add(rect1);
									reck2.add(rect2);
								} else {
									reck1.add(rect2);
									reck2.add(rect1);
								}
								reck3.add(temp);
								i = 1005;
							}
						}
					}
				
				/*if (0.85<comp1&&comp1<1.15&&0.85<comp2&&comp2<1.15&&0.85<comp3&&comp3<1.15&&rect1.width>20&&comp4==true){
					System.out.println("Target Found");
					//Break out of for loop
					b=boundRect.size()+100000;
					j=boundRect.size()+100000;
					//Draw confirmed rectangles
					Imgproc.rectangle( source, rect2.tl(), rect2.br(), new Scalar(0,0,255), 2, 8, 0 );
					Imgproc.rectangle( source, rect1.tl(), rect1.br(), new Scalar(0,0,255), 2, 8, 0 );
					Imgproc.rectangle( source, temp.tl(),  temp.br(),  new Scalar(0,255,0), 2, 8, 0);
					//Create new variables for correct boxes
					confRectFull=temp; 
					if(rect1.y<rect2.y){
						confRectTop=rect1;
						confRectBot=rect2;
					} else{
						confRectTop=rect2;
						confRectBot=rect1;
					}
				}else {
					//NO Target Found!!!
//					System.out.println("No Target");
					//NOTE: Use something better than print to alert operator that no target was found
					//and do something to continue, like try the cycle again OR change to find the best target
				}*/
			
				
				}
			
			/*else{
				if(boundRect.get(b) == null || boundRect.get(j) == null){
					//Rectangle is null
					//System.out.println("Rectangle was null");
				} else if(b!=j && j>b){
					//bad combo
					//System.out.println("Bad combo of rectangles, try triangles");
				}
			}*/
			}
			//System.out.println("this is confirmation");
		}
		
		for (Integer i=0; i>poss.length; i++) {
			if (i == 0 && poss[i] != null) {
				confirmed = poss[i];
				welike = i;
			} else if (i == 0 && poss[i] == null) {
				confirmed = 0;
				welike = i;
			} else if (poss[i] != null) {
				if (1 - Math.abs(poss[i] - 1) > 1 - Math.abs(confirmed - 1)) {
					confirmed = poss[i];
					welike = i;
				}
			} else {
				break;
			}
		}
		
		if (welike != null) {
			System.out.println("Target Found");
			if (welike < reck1.size()  && welike < reck2.size() && welike < reck3.size()){
				Rect rect1 = reck1.get(welike);
				Rect rect2 = reck2.get(welike);
				Rect temp = reck3.get(welike);
				//Draw confirmed rectangles
				Imgproc.rectangle(source, rect2.tl(), rect2.br(), new Scalar(0,0,255), 2, 8, 0);
				Imgproc.rectangle(source, rect1.tl(), rect1.br(), new Scalar(0,0,255), 2, 8, 0);
				Imgproc.rectangle(source, temp.tl(),  temp.br(),  new Scalar(0,255,0), 2, 8, 0);
				//Create new variables for correct boxes
				confRectFull = temp; 
				confRectTop = rect1;
				confRectBot = rect2;
		
		//Post size of boundRect arraylist
//		SmartDashboard.putInt("boundrect array size", boundRect.size());
			}
		}
	}
	
	public void viewShooter() {
		//put the processed image with rectangles on smartdashboard
		cvSource.putFrame(source);
	}
	
	public Double getDistance() {
		if(confRectTop == null) {
			return  null;
		}
		double fullYFOV = 41.8;
		double pixelHeight = 480;
		double halfFOV = fullYFOV / 2;
		double distanceFromZero = 51;
		
		
		//get y of middle of rect
		/*Point right = confRectTop.br();
		Point left = confRectTop.tl();
		double parthalf = right.y-left.y;
		parthalf = parthalf/2;
		double half = left.y + parthalf;
		
		double centerhalf =  half-240;
		half = Math.abs(half);
		double pixelRatioVerticle = centerhalf / (pixelHeight/2);
		double angle = halfFOV * pixelRatioVerticle;*/
		Point bot = confRectTop.br();
		Point top = confRectTop.tl();
		
		//get y of middle of rect 
		double parthalf = bot.y-top.y;
		parthalf = parthalf/2;
		double half = top.y + parthalf;
		
		double centerhalf =  half-(pixelHeight/2);
		double pixelRatioHorizontal = centerhalf / (pixelHeight/2);
		double angle = halfFOV * pixelRatioHorizontal;
		double angle_Abs = Math.abs(angle);
		double angle_Radians = angle_Abs*Math.PI*2/360;
		
		double distance = distanceFromZero/Math.tan(angle_Radians);
		
		return new Double(distance);
	}
	
	public Double getAngle() {
		if (confRectTop == null) {
			return  null;
		}
		double fullXFOV = 53.14;
		double pixelWidth = 640;
		double halfFOV = fullXFOV / 2;
		
		Point bot = confRectTop.br();
		Point top = confRectTop.tl();
		
		//get x of middle of rect 
		double parthalf = bot.x-top.x;
		parthalf = parthalf/2;
		double half = top.x + parthalf;
		
		double centerhalf =  half-(pixelWidth/2);
		double pixelRatioHorizontal = centerhalf / (pixelWidth/2);
		double angle = halfFOV * pixelRatioHorizontal;
		
		return new Double(angle);
	}
	
} 