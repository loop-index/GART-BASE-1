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
			// Get the UsbCamera from CameraServer
			// Set the resolution
			Robot.camera.setResolution(320, 240);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream
					= CameraServer.getInstance().putVideo("Vision", 320, 240);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();
			Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
			List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
			Mat hierarchy = new Mat();
			boolean ran = false;
			
			int R = prefs.getInt("R", 0);
			int G = prefs.getInt("G", 0);
			int B = prefs.getInt("B", 0);
			int E = prefs.getInt("E", 20);
			int biggestBlock = 0;
			
			SmartDashboard.putNumber("R", R);
			SmartDashboard.putNumber("G", G);
			SmartDashboard.putNumber("B", B);
			SmartDashboard.putNumber("E", E);
			
			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
//			while (!Thread.interrupted()) {
			while (true){
				biggestBlock = 0;
				prefs.putInt("R", R);
				prefs.putInt("G", G);
				prefs.putInt("B", B);
				prefs.putInt("E", E);
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
				E = (int) SmartDashboard.getNumber("E", 20);
				
//				//user code
				Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV);
				Core.inRange(mat, new Scalar(R - E, G - E, B - E), new Scalar(R + E, G + E, B + E), mat);
//				Imgproc.erode(mat, mat, kernel);
				Imgproc.dilate(mat, mat, kernel);
//				Imgproc.dilate(mat, mat, kernel);
				
				Imgproc.findContours(mat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
				
				for (int i = 0; i < contours.size(); i++){
					Rect boundRect = Imgproc.boundingRect(contours.get(i));
//					float ratio = (float) (Imgproc.contourArea(contours.get(i))/(boundRect.width*boundRect.height));
					if (Imgproc.contourArea(contours.get(i)) > 0){
						biggestBlock = (int) Imgproc.contourArea(contours.get(i));
						centerX = boundRect.x + (boundRect.width / 2);
						centerY = boundRect.y + (boundRect.height / 2);
//						Imgproc.drawContours(mat, contours, i, new Scalar(255, 0, 0), 1);
//						Imgproc.circle(mat, new Point(centerX, centerY), 40, new Scalar (255,0,0));
//						SmartDashboard.putNumber("size", Imgproc.contourArea(contours.get(i)));
					}
				}
				SmartDashboard.putString("Coordinates", centerX + ", " + centerY);
				SmartDashboard.putNumber("largest size", biggestBlock);

				// Give the output stream a new image to display
				outputStream.putFrame(mat);
				RobotMap.drivebase.followX(25);
			}
//		});
//		m_visionThread.setDaemon(true);
//		m_visionThread.start();
			
//		Power Cube: 0 - 170 - 200 - E: 50
	}
}
