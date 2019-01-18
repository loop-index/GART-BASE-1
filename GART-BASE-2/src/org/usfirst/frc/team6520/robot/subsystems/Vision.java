package org.usfirst.frc.team6520.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team6520.robot.Robot;
import org.usfirst.frc.team6520.robot.RobotMap;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
//	Thread m_visionThread;
	double centerX = 0, centerY = 0;
	Preferences prefs = Preferences.getInstance();

	public void vision() {
//		m_visionThread = new Thread(() -> {
			Robot.camera.setResolution(320, 240);

			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream
					= CameraServer.getInstance().putVideo("Vision", 320, 240);
			
			Mat mat = new Mat();
			Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
			List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//			List<String> coords = new ArrayList<String>();
			String coords = "";
			int count = 0;
			Mat hierarchy = new Mat();
			boolean ran = false;
			
			int R = prefs.getInt("R", 0);
			int G = prefs.getInt("G", 0);
			int B = prefs.getInt("B", 0);
			int RE = prefs.getInt("RE", 20);
			int GE = prefs.getInt("GE", 20);
			int BE = prefs.getInt("BE", 20);
			int biggestBlock = 0;
			
			SmartDashboard.putNumber("R", R);
			SmartDashboard.putNumber("G", G);
			SmartDashboard.putNumber("B", B);
			SmartDashboard.putNumber("RE", RE);
			SmartDashboard.putNumber("GE", GE);
			SmartDashboard.putNumber("BE", BE);
			
//			while (!Thread.interrupted()) {
			while (true){
//				coords.removeAll(coords);
				biggestBlock = 0;
				prefs.putInt("R", R);
				prefs.putInt("G", G);
				prefs.putInt("B", B);
				prefs.putInt("RE", RE);
				prefs.putInt("GE", GE);
				prefs.putInt("BE", BE);
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
//					return 0;
				}
				
				R = (int) SmartDashboard.getNumber("R", 255);
				G = (int) SmartDashboard.getNumber("G", 255);
				B = (int) SmartDashboard.getNumber("B", 255);
				RE = (int) SmartDashboard.getNumber("RE", 20);
				GE = (int) SmartDashboard.getNumber("GE", 20);
				BE = (int) SmartDashboard.getNumber("BE", 20);
				
//				//user code
				Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
				Core.inRange(mat, new Scalar(R - RE, G - GE, B - BE), new Scalar(R + RE, G + GE, B + BE), mat);
				Imgproc.dilate(mat, mat, kernel);
				
				Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

				
				double Xsum = 0;
				System.out.println("loop" + count);
				count = 0;
				System.out.println("loop" + count);
				for (int i = 0; i < contours.size(); i++){
					Rect boundRect = Imgproc.boundingRect(contours.get(i));
					float ratio = (float) (Imgproc.contourArea(contours.get(i))/(boundRect.width*boundRect.height));
					if (Imgproc.contourArea(contours.get(i)) > 300
							&& ratio > 0.3){
						biggestBlock = (int) Imgproc.contourArea(contours.get(i));
						centerX = boundRect.x + (boundRect.width / 2);
						centerY = boundRect.y + (boundRect.height / 2);
//						coords += (centerX + ", " + centerY);
//						Xsum += centerX;
						count++;
						coords = "" + count;
//						Imgproc.putText(mat, "" + centerX, new Point(centerX, centerY - 30), 0, 0.5, new Scalar(255, 255, 255));
//						Imgproc.drawContours(mat, contours, i, new Scalar(255, 0, 0), 1);
//						Imgproc.circle(mat, new Point(centerX, centerY), 40, new Scalar (255,0,0));
//						SmartDashboard.putNumber("size", Imgproc.contourArea(contours.get(i)));
					}
//					SmartDashboard.putString("coords", coords);
				}
				SmartDashboard.putString("Coordinates", centerX + ", " + centerY);

				// Give the output stream a new image to display
				outputStream.putFrame(mat);
//				RobotMap.drivebase.followX(25);
			}
//		});
//		m_visionThread.setDaemon(true);
//		m_visionThread.start();
			
//		Power Cube: 0 - 170 - 200 - E: 50
	}
}