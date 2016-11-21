package projects.multipath.advanced;

import java.awt.Font;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

public class Path {

	public int id = 0;
	public List<Integer> vertexList = new ArrayList<Integer>();

	public void addVertex(Integer v){
		vertexList.add(v);
	}
	
	public void insertVertex(Integer v){
		vertexList.add(0, v);
	}
	
	public void printPath(Graph g){
		for(int i = 0; i < vertexList.size(); i ++){
			g.idVertexMap.get(vertexList.get(i)).printVertex();
			System.out.print("|");
		}
		System.out.println();
	}

	public void drawPaths(Graphics2D g2d, Graph g, int scale, int number) {
		int currentEdge = 0;

		// Draw edges
		g2d.setPaint(new java.awt.Color(0x202020));
		while(currentEdge < vertexList.size() - 1){
			Vertex startVertex = g.idVertexMap.get(vertexList.get(currentEdge));
			Vertex endVertex = g.idVertexMap.get(vertexList.get(currentEdge + 1));
			int sx = (int)(startVertex.getX());
			int sy = (int)(startVertex.getY());
			int ex = (int)(endVertex.getX());
			int ey = (int)(endVertex.getY());
			g2d.drawLine((sx) * scale, (sy)*scale, (ex)*scale, (ey)*scale);
			currentEdge ++;
		}

	}

	public void drawRobots(Graphics2D g2d, Graph g, int scale, int number) {
		// Draw beginning location as a dot
		Vertex firstVertex = g.idVertexMap.get(vertexList.get(0));
		g2d.setPaint(new java.awt.Color(0x6495ed));
		//g2d.drawOval((int)(firstVertex.getX()) * scale - scale/2 + 1, (int)(firstVertex.getY()) * scale - scale/2 + 1, scale - 2, scale - 2);
		g2d.fillOval((int)(firstVertex.getX()) * scale - scale/2 + 1, (int)(firstVertex.getY()) * scale - scale/2 + 1, scale - 2, scale - 2);
		
		Font f = new Font(g2d.getFont().getName(), Font.BOLD, 5);
		g2d.setPaint(new java.awt.Color(0xF0F0F0));
		g2d.setFont(f);
		String s = "" + number;
		g2d.drawString(s, (int)(firstVertex.getX()*scale - scale/3) + (number>9?0:2),(int)((firstVertex.getY())*scale + 2));

	}
}

