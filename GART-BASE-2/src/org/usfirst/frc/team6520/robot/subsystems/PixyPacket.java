package org.usfirst.frc.team6520.robot.subsystems;

public class PixyPacket {
	public int X;
	public int Y;
	public int Width;
	public int Height;
	public int Sig;
	public int Area;
	
	public PixyPacket() {
		X = 0;
		Y = 0;
		Width = 0;
		Height = 0;
		Sig = 0;
		Area = 0;
	}
	
	public void setX(int x) {
		X = x;
	}
	public void setY(int y) {
		Y = y;
	}
	public void setWidth(int w) {
		Width = w;
	}
	public void setHeight(int h) {
		Height = h;
	}
	public void setSig(int sig){
		Sig = sig;
	}
	
	public void setArea(int a){
		Area = a;
	}
}