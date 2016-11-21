package projects.multipath.advanced;

import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;

public class Vertex {
	
	public int id;
	protected Point2D point = null;
	public int usedInPath = 0;

	public static final int STYLE_START = 1; 
	public static final int STYLE_END = 2; 
	
	public Vertex(Point2D point, int id){
		this.point = point;
		this.id = id;
	}
	
	public Vertex(Point2D point) {
		this.point = point;
	}

	public Vertex(int id){
		this.id = id;
	}

	public double getX(){
		return point.getX();
	}
	
	public double getY(){
		return point.getY();
	}
	
	public int getIX(){
		return (int)(getX());
	}
	
	public int getIY(){
		return (int)(getY());
	}

	public void print(){
		System.out.print("" + id);
	}
	
	public void printVertex(){
		print();
		if(point != null){
			System.out.print("(" + getIX()+"," + getIY() + ")");
			// System.out.print("" + getIX()+"," + getIY() + "");
		}
	}
	
	public void printVertex(StringBuffer buffer){
		if(point != null){
			buffer.append("(" + getIX()+"," + getIY() + ")");
		}
	}

	public void draw(Graphics2D g2d, int scale, int style, int number) {
		g2d.setPaint(new java.awt.Color(0xF0F0F0));
		g2d.fillOval((int)(this.point.getX()*scale - scale/2),(int)(this.point.getY()*scale - scale/2), scale, scale);

		switch(style){
		case STYLE_START: 
			g2d.setPaint(new java.awt.Color(0xD04040));
			break;
		case STYLE_END: 
			g2d.setPaint(new java.awt.Color(0x4040D0));
			break;
			default:;
		}

		g2d.drawOval((int)(this.point.getX()*scale - scale/2),(int)(this.point.getY()*scale - scale/2), scale, scale);
		
		Font f = new Font(g2d.getFont().getName(), Font.BOLD, 5);
		g2d.setFont(f);
		String s = "" + number;
		g2d.drawString(s, (int)(this.point.getX()*scale - scale/3) + (number>9?0:2),(int)(this.point.getY()*scale + 2));
	}

}
