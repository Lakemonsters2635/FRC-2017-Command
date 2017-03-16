package org.usfirst.frc.team2635.robot.model;

import java.text.SimpleDateFormat;
import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearVision extends Vision {
	Rect confRectRight;
	Rect confRectLeft;
	Rect confRectFull;
	ArrayList<Rect> reck1;
	ArrayList<Rect> reck2;
	ArrayList<Rect> reck3;
	
	Double confirmed;
	Integer welike;
	
	public GearVision(UsbCamera camera){
		super(camera);
	}
	
	public void confirmBox(){
		Double[] poss = new Double[999];
		reck1 = new ArrayList<Rect>();
		reck2 = new ArrayList<Rect>();
		reck3 = new ArrayList<Rect>();
		for( Integer b = 0; b < boundRect.size(); b++ ){
			for (Integer j = 1; j< boundRect.size(); j++){
				//Integer j = b;
				if (boundRect.get(b) != null && boundRect.get(j) != null&&b!=j&&j>b){
				
					Rect rect1 = boundRect.get(b);
					Rect rect2 = boundRect.get(j);
					Rect temp;
					Double leftH;
					Double leftW;
					Double rightH;
					Double rightW;
					Integer check;
					//Post height of rectangles for debug
					SmartDashboard.putInt("rect1y",rect1.y);
					SmartDashboard.putInt("rect2y", rect2.y);
					//Decide which rectangle is left or right
					if(rect1.x<rect2.x){
						leftH = (double) rect1.height;
						leftW = (double) rect1.width;
						rightH = (double) rect2.height;
						rightW = (double) rect2.width;
						temp = Imgproc.boundingRect(new MatOfPoint(rect2.br(),rect1.tl()));
						//Uncomment to see what box is being tested
						//Imgproc.rectangle( source, rect1.tl(), rect1.br(), new Scalar(0,0,255), 2, 8, 0 );
						check = 0;
					}
					else {
						leftH = (double) rect2.height;
						leftW = (double) rect2.width;
						rightH = (double) rect1.height;
						rightW = (double) rect1.width;
						temp = Imgproc.boundingRect(new MatOfPoint(rect1.br(),rect2.tl()));
						//Uncomment to see what box is being tested
						//Imgproc.rectangle( source, rect2.tl(), rect2.br(), new Scalar(0,0,255), 2, 8, 0 );
						check = 1;
					}
					//Create variables to be used for confirmation
					
					//Do checks on rectangle pair
					Double comp1;
					if(leftH>=rightH){
						comp1 = leftH/rightH;
					} else {
						comp1 = rightH/leftH;
					}
					Double comp2;
					if(leftH>=rightH){
					comp2 = leftW/rightW;
					} else {
						comp2 = rightW/leftW;
					}
					
					Double comp3;
					if(check==0){
						Double comp31 =  (rect1.width*4.375);
						Double comp32 = rect1.x+comp31;
						comp3 = comp32/rect2.x;
						//SmartDashboard.putDouble("comp31", comp31);
						//SmartDashboard.putDouble("comp32", comp32);
						//SmartDashboard.putDouble("left.width", rect1.width);
						//SmartDashboard.putDouble("right.width", rect2.width);
						//SmartDashboard.putDouble("rect1.x", rect1.x);
						//SmartDashboard.putDouble("rect2.x", rect2.x);
						
					} else{
						Double comp31 = (rect2.width*4.375);
						Double comp32 = rect2.x+comp31;
						comp3 = comp32/rect1.x;
						//SmartDashboard.putDouble("comp31", comp31);
						//SmartDashboard.putDouble("comp32", comp32);
						//SmartDashboard.putDouble("right.width", rect1.width);
						//SmartDashboard.putDouble("left.width", rect2.width);
						//SmartDashboard.putDouble("rect2.x", rect2.x);
						//SmartDashboard.putDouble("rect1.x", rect1.x);
					}
					Double comp4;
					Point b1 = rect1.br();
					Point b2 = rect2.br();
					comp4 = b1.y/b2.y;
					//Post used variables
					
					//Post results of checks
					SmartDashboard.putDouble("comp1", comp1);
					SmartDashboard.putDouble("comp2", comp2);
					
					SmartDashboard.putDouble("comp3", comp3);
					SmartDashboard.putDouble("comp4", comp4);
					if (rect1.height>20&&rect2.height>20&&.7<comp1&&1.3>comp1&&.7<comp2&&1.3>comp2&&.7<comp3&&1.3>comp3&&.7<comp4&&1.3>comp4){
						Double done = 1 - (1 * Math.abs(4 - (comp1+comp2+comp3+comp4)));
						for(int i=0;i<999;i++){
							if(poss[i]==null){
								poss[i]=done;
								if(rect1.x<rect2.x){
									reck1.add(rect1);
									reck2.add(rect2);
								} else{
									reck1.add(rect2);
									reck2.add(rect1);
								}
								reck3.add(temp);
								i = 1005;
							}
						}
					}
				}
			}
		}
		
		for (Integer i=0;i<poss.length;i++){
			
			if(i==0&&poss[i]!=null){
				confirmed = poss[i];
				welike = i;
			} else if(i==0&&poss[i]==null){
				confirmed = 0.0;
				welike = i;
			} else if(poss[i]!=null){
				if(1-Math.abs(poss[i]-1)>1-Math.abs(confirmed-1)){
					confirmed = poss[i];
					welike = i;
				}
				
			} else{
				break;
			}
		}
		if(welike!=null){
			//System.out.println("Target Found, welike:" +welike);
			if (welike < reck1.size()  && welike < reck2.size() && welike < reck3.size()){
				Rect rect1 = reck1.get(welike);
				Rect rect2 = reck2.get(welike);
				Rect temp = reck3.get(welike);
				//Draw confirmed rectangles
				Imgproc.rectangle( source, rect2.tl(), rect2.br(), new Scalar(0,0,255), 2, 8, 0 );
				Imgproc.rectangle( source, rect1.tl(), rect1.br(), new Scalar(0,0,255), 2, 8, 0 );
				Imgproc.rectangle( source, temp.tl(),  temp.br(),  new Scalar(0,255,0), 2, 8, 0);
				//Create new variables for correct boxes
				confRectFull=temp; 
				if(rect1.x<rect2.x){
					confRectLeft=rect1;
					confRectRight=rect2;
				} else{
					confRectLeft=rect2;
					confRectRight=rect1;
				}
			}
		}
	}
	 
	public void viewShooter(){
   		//Draw Crosshairs
		Point line11 = new Point(0,240);
		Point line12 = new Point(620,240);
		Point line21 = new Point(320,0);
		Point line22 = new Point(320,480);
		Imgproc.line(source, line11, line12, new Scalar(255,255,255));
		Imgproc.line(source, line21, line22, new Scalar(255,255,255));
		//Save Image
		currentdatehour = new SimpleDateFormat("MM/dd/yyy HH:mm:ss:ms").format(new java.util.Date());
		Imgcodecs.imwrite("C:\\Users\\Robbie Robot\\Vision Log\\"+currentdate+"\\"+currentdatehour+".jpg", source);
		//put the processed image with rectangles on smartdashboard
		cvSource.putFrame(source);
	}
	
	public Double getDistance(){
		if(confRectRight == null)
		{
			return  null;
		}
		double fullYFOV = 41.8;
		double pixelHeight = 480;
		double halfFOV = fullYFOV / 2;
		double distanceFromZero = 16;
		
		
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
		Point left = confRectRight.br();
		Point right = confRectRight.tl();
		
		//get y of middle of rect 
		double parthalf = left.y-right.y;
		parthalf = parthalf/2;
		double half = right.y + parthalf;
		
		double centerhalf =  half-(pixelHeight/2);
		double pixelRatioHorizontal = centerhalf / (pixelHeight/2);
		double angle = halfFOV * pixelRatioHorizontal;
		double angle_Abs = Math.abs(angle);
		//System.out.println("angle_Abs: " + angle_Abs);
		double angle_Radians = angle_Abs*Math.PI*2/360;
		
		double distance = distanceFromZero/Math.tan(angle_Radians);
		
		return new Double(distance);
	}
	
	public Double getAngle(){
		if(confRectRight == null||confRectLeft == null)
		{
			return  null;
		}
		double fullXFOV = 53.14;
		double pixelWidth = 640;
		double halfFOV = fullXFOV / 2;
		
		Point left = confRectRight.br();
		Point right = confRectRight.tl();
		
		//get x of middle of rect 
		double parthalf = left.x-right.x;
		parthalf = parthalf/2;
		double half = right.x + parthalf;
		
		double centerhalf =  half-(pixelWidth/2);
		double pixelRatioHorizontal = centerhalf / (pixelWidth/2);
		double angle = halfFOV * pixelRatioHorizontal;
		
		return  new Double(angle);
	}
}