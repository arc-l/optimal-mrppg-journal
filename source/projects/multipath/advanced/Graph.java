package projects.multipath.advanced;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.io.Serializable;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.TreeSet;

public class Graph implements Serializable{
	private static final long serialVersionUID = 1L;
	
	// Vertex set
	public Set<Vertex> verticeSet = new HashSet<Vertex>();
	public Vertex[] vertices = null;
	
	// Vertex id set
	public SortedMap<Integer, Vertex> idVertexMap = new TreeMap<Integer, Vertex>();
	
	// Adjacency list (as a set) map
	public Map<Integer, Set<Integer>> adjacencySetMap = new HashMap<Integer, Set<Integer>>();

	// For rectangle
	int rows, columns; 
	
	/**
	 * Create a 2D grid with some rows and columns
	 * @param rows
	 * @param cols
	 * @return
	 */
	public static Graph create2DGridGraph(int rows, int cols, boolean update){
		Graph g = new Graph();
		g.rows = rows;
		g.columns = cols;
		// Create vertices
		for(int i = 0; i < rows; i ++){
			for(int j = 0; j < cols; j ++){
				Vertex v = new Vertex(new Point2D.Double(j, i));
				v.id = g.getId(j, i);
				g.verticeSet.add(v);
				g.idVertexMap.put(v.id, v);
			}
		}

		for(int i = 0; i < rows; i ++){
			for(int j = 0; j < cols; j ++){
				int id = g.getId(j, i);
				Set<Integer> nbrs = new TreeSet<Integer>();
				g.adjacencySetMap.put(id, nbrs);
				nbrs.add(id);
				nbrs.add(id - 1);
				nbrs.add(id + 1);
				nbrs.add(id - cols);
				nbrs.add(id + cols);
				
				if(i == 0){
					nbrs.remove(id - cols);
				}
				if(j == 0){
					nbrs.remove(id - 1);
				}
				if(i == rows - 1){
					nbrs.remove(id + cols);
				}
				if(j == cols - 1){
					nbrs.remove(id + 1);
				}
			}
		}
		g.vertices = g.verticeSet.toArray(new Vertex[0]);
		if(!update){
			return g;
		}
		return Graph.convertGraph(g, new int[0], new int[0]);
	}

	/**
	 * Create a 2D grid with some rows and columns
	 * @param rows
	 * @param cols
	 * @return
	 */
	public static Graph create2DGridGraph8Connected(int rows, int cols, boolean update){
		Graph g = new Graph();
		g.rows = rows;
		g.columns = cols;
		// Create vertices
		for(int i = 0; i < rows; i ++){
			for(int j = 0; j < cols; j ++){
				Vertex v = new Vertex(new Point2D.Double(j, i));
				v.id = g.getId(j, i);
				g.verticeSet.add(v);
				g.idVertexMap.put(v.id, v);
			}
		}

		for(int i = 0; i < rows; i ++){
			for(int j = 0; j < cols; j ++){
				int id = g.getId(j, i);
				Set<Integer> nbrs = new TreeSet<Integer>();
				g.adjacencySetMap.put(id, nbrs);
				nbrs.add(id);
				nbrs.add(id - 1);
				nbrs.add(id + 1);
				nbrs.add(id - cols);
				nbrs.add(id + cols);
				nbrs.add(id - cols - 1);
				nbrs.add(id - cols + 1);
				nbrs.add(id + cols - 1);
				nbrs.add(id + cols + 1);
				
				if(i == 0){
					nbrs.remove(id - cols);
					nbrs.remove(id - cols - 1);
					nbrs.remove(id - cols + 1);
				}
				if(j == 0){
					nbrs.remove(id - 1);
					nbrs.remove(id - cols - 1);
					nbrs.remove(id + cols - 1);
				}
				if(i == rows - 1){
					nbrs.remove(id + cols);
					nbrs.remove(id + cols - 1);
					nbrs.remove(id + cols + 1);
				}
				if(j == cols - 1){
					nbrs.remove(id + 1);
					nbrs.remove(id - cols + 1);
					nbrs.remove(id + cols + 1);
				}
			}
		}
		g.vertices = g.verticeSet.toArray(new Vertex[0]);
		if(!update){
			return g;
		}
		return Graph.convertGraph(g, new int[0], new int[0]);
	}

	/**
	 * Create a grid with some vertices removed, keeping the graph connected. 
	 * @param rows
	 * @param cols
	 * @param percentObstacle
	 * @return
	 */
	public static Graph createGeneric2DGridGraphWithHoles8Connected(int rows, int cols, double percentObstacle){
		Graph g = Graph.create2DGridGraph8Connected(rows, cols, false);
		int verticesToRemove = (int)(rows*cols*percentObstacle);
		while(verticesToRemove > 0){
			int yToRemove =  (int)(Math.random()*rows);
			int xToRemove =  (int)(Math.random()*cols);
			int vId = g.getId(xToRemove, yToRemove);
			if(g.idVertexMap.containsKey(vId)){
				if(g.isConnectedWithoutVertex(vId)){
					g.removeVertex(xToRemove, yToRemove);
					verticesToRemove --;
				}
			}
		}
		g.vertices = g.verticeSet.toArray(new Vertex[0]);
		return g;
	}

	/**
	 * Create a grid with some vertices removed, keeping the graph connected. 
	 * @param rows
	 * @param cols
	 * @param percentObstacle
	 * @return
	 */
	public static Graph createGeneric2DGridGraphWithHoles(int rows, int cols, double percentObstacle){
		Graph g = Graph.create2DGridGraph(rows, cols, false);
		int verticesToRemove = (int)(rows*cols*percentObstacle);
		while(verticesToRemove > 0){
			int yToRemove =  (int)(Math.random()*rows);
			int xToRemove =  (int)(Math.random()*cols);
			int vId = g.getId(xToRemove, yToRemove);
			if(g.idVertexMap.containsKey(vId)){
				if(g.isConnectedWithoutVertex(vId)){
					g.removeVertex(xToRemove, yToRemove);
					verticesToRemove --;
				}
			}
		}
		g.vertices = g.verticeSet.toArray(new Vertex[0]);
		return g;
	}
	
	/**
	 * Randomly pick some start and goal locations. This function works well only when number is
	 * relatively small compared to the number of vertices. 
	 * @param g
	 * @param number
	 * @return
	 */
	public static int[][] getRandomStartGoal(Graph g, int number){
		Set<Integer> startSet = new HashSet<Integer>();
		Set<Integer> goalSet = new HashSet<Integer>();
		int ret[][] = new int[2][number];
		for(int i = 0; i < number; i ++){
			while(true){
				if(g.columns != 0){
					// Create start
					int x = (int)(Math.random()*g.columns);
					int y = (int)(Math.random()*g.rows);
					int id = g.getId(x, y);
					if(startSet.contains(id) || g.idVertexMap.get(id) == null) continue;
					int x2 = (int)(Math.random()*g.columns);
					int y2 = (int)(Math.random()*g.rows);
					int id2 = g.getId(x2, y2);
					if(goalSet.contains(id2) || g.idVertexMap.get(id2) == null) continue;
					startSet.add(id);
					goalSet.add(id2);
					ret[0][i] = id;
					ret[1][i] = id2;
					break;
				}
				else {
					// Create start
					int id = (int)(Math.random()*g.vertices.length);
					if(startSet.contains(id) || g.idVertexMap.get(id) == null) continue;
					int id2 = (int)(Math.random()*g.vertices.length);
					if(goalSet.contains(id2) || g.idVertexMap.get(id2) == null) continue;
					startSet.add(id);
					goalSet.add(id2);
					ret[0][i] = id;
					ret[1][i] = id2;
					break;
				}
			}
		}
		return ret;
	}
	
	/**
	 * Randomly pick start and goals for many agents (~ number of vertices)
	 * @param g
	 * @param number
	 * @return
	 */
	public static int[][] getRandomStartGoalMany(Graph g, int number){
		List<Integer> startList = new LinkedList<Integer>();
		List<Integer> goalList = new LinkedList<Integer>();
		for(int i = 0; i <g.vertices.length; i++){
			startList.add(g.vertices[i].id);
			goalList.add(g.vertices[i].id);
		}
		
		int ret[][] = new int[2][number];
		for(int i = 0; i < number; i ++){
			while(true){
				ret[1][i] = startList.remove((int)(Math.random()*startList.size()));
				ret[0][i] = goalList.remove(0);
				break;
			}
		}
		return ret;
	}

	/**
	 * Convert the graph so that all vertices are reorderd with consecutive ids that
	 * starts with 0. 
	 * @param graph
	 * @param start
	 * @param goal
	 * @return
	 */
	protected static Graph convertGraph(Graph graph, int start[], int goal[]){
		Graph og = new Graph();
		Map<Integer, Integer> vIdIndexMap = new HashMap<Integer, Integer>();
		SortedMap<Integer, Vertex> indexVertexMap = new TreeMap<Integer,Vertex>();
		for(int i = 0; i < graph.vertices.length; i ++){
			indexVertexMap.put(graph.vertices[i].id, graph.vertices[i]);
		}
		Vertex vs[] = indexVertexMap.values().toArray(new Vertex[0]);
		for(int i = 0; i < graph.vertices.length; i ++){
			vIdIndexMap.put(vs[i].id, i);
		}
		og.vertices = new Vertex[graph.vertices.length];
		for(int i = 0; i < graph.vertices.length; i ++){
			og.vertices[i] = new Vertex(vs[i].point);
			og.vertices[i].id = i;
			og.idVertexMap.put(i, og.vertices[i]);
			og.verticeSet.add(og.vertices[i]);
			Set<Integer> adjSet = new TreeSet<Integer>();
			og.adjacencySetMap.put(i, adjSet);
			Integer[] adjs = graph.adjacencySetMap.get(vs[i].id).toArray(new Integer[0]);
			for(int j = 0; j < adjs.length; j ++){
				adjSet.add(vIdIndexMap.get(adjs[j]));
			}
		}
		
		for(int i = 0; i < start.length; i ++){
			start[i] = vIdIndexMap.get(start[i]);
			goal[i] = vIdIndexMap.get(goal[i]);
		}
		
		return og;
	}
	
	public static long EDGE_MULTIPLIER = 100000000L;
	public Set<Long> getEdgeSet(){
		Set<Long> edgeSet = new HashSet<Long>();
		for(int i = 0; i < vertices.length; i ++){
			for(int j: adjacencySetMap.get(i)){
				if(i == j) continue;
				edgeSet.add(getEdgeId(i, j));
			}
		}
		return edgeSet;
	}
	
	public static long getEdgeId(int u, int v){
		return u>v? v*EDGE_MULTIPLIER + u: u*EDGE_MULTIPLIER + v;
	}
	
	public static int getFirstVertexOfEdge(long edge){
		return (int)(edge/EDGE_MULTIPLIER);
	}
	
	public static int getSecondVertexOfEdge(long edge){
		return (int)(edge%EDGE_MULTIPLIER);
	}
	

	/**
	 * For saving/retrieving the graph
	 * @param out
	 * @throws IOException
	 */
	private void writeObject(ObjectOutputStream out) throws IOException{
		// Gather vertice info and write to object
		int v[][] = new int[vertices.length][13];
		for(int i = 0; i <vertices.length; i ++){
			v[i][0] = vertices[i].id;
			v[i][1] = vertices[i].getIX();
			v[i][2] = vertices[i].getIY();
			
			// Adj list
			Integer adj[] = adjacencySetMap.get(vertices[i].id).toArray(new Integer[0]);
			v[i][3] = adj.length; 
			for(int j = 0; j < adj.length; j ++){
				v[i][4 + j] = adj[j];
			}
		}
		out.writeObject(v);
		out.writeInt(rows);
		out.writeInt(columns);
	}
	
	/**
 	 * For saving/retrieving the graph
	 * @param in
	 * @throws IOException
	 * @throws ClassNotFoundException
	 */
	private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException{
		verticeSet = new HashSet<Vertex>();
		idVertexMap = new TreeMap<Integer, Vertex>();
		adjacencySetMap = new HashMap<Integer, Set<Integer>>();
		int v[][] = (int[][])in.readObject();
		this.vertices = new Vertex[v.length];
		for(int i = 0; i < v.length; i ++){
			vertices[i] = new Vertex(new Point2D.Double(v[i][1], v[i][2]));
			vertices[i].id = v[i][0];
			verticeSet.add(vertices[i]);
			idVertexMap.put(vertices[i].id, vertices[i]);
			Set<Integer> adjSet = new TreeSet<Integer>();
			adjacencySetMap.put(vertices[i].id, adjSet);
			for(int j = 0; j <v[i][3]; j ++){
				adjSet.add(v[i][4 + j]); 
			}
		}
		// vertices = verticeSet.toArray(new Vertex[0]);
		rows = in.readInt();
		columns = in.readInt();
	}
	
	/**
	 * Deleting a vertex from a grid graph, keeping the connectivity
	 * @param x
	 * @param y
	 */
	private void removeVertex(int x, int y){
		int id = getId(x, y);
		Vertex v = idVertexMap.get(id);
		
		// Remove the vertex
		verticeSet.remove(v);
		
		// Remove this from idVertexMap 
		idVertexMap.remove(id);
		
		// Remove v from all possible adjacency maps. For now just do brute force
		adjacencySetMap.remove(id);
		Integer ids[] = adjacencySetMap.keySet().toArray(new Integer[0]);
		for(int i = 0; i < ids.length; i ++){
			Set<Integer> idSet =  adjacencySetMap.get(ids[i]);
			idSet.remove(id);
			if(idSet.size() == 0){
				adjacencySetMap.remove(ids[i]);
			}
		}
	}
	
	/**
	 * Deleting a vertex, keeping the connectivity
	 * @param v
	 */
	public void removeVertex(Vertex v){
		// Remove the vertex
		verticeSet.remove(v);
		
		// Remove this from idVertexMap 
		idVertexMap.remove(v.id);
		
		// Remove v from all possible adjacency maps. For now just do brute force
		adjacencySetMap.remove(v.id);
		Integer ids[] = adjacencySetMap.keySet().toArray(new Integer[0]);
		for(int i = 0; i < ids.length; i ++){
			Set<Integer> idSet =  adjacencySetMap.get(ids[i]);
			idSet.remove(v.id);
			if(idSet.size() == 0){
				adjacencySetMap.remove(ids[i]);
			}
		}
	}
	
	//////////////////////////////////////////////////
	// For constructing a new graph 
	//////////////////////////////////////////////////
	
	/**
	 *  Add a vertex 
	 * @param v
	 */
	public void addVertex(Vertex v){
		this.verticeSet.add(v);
		this.idVertexMap.put(v.id, v);
	}
	
	
	private int getId(int x, int y){
		return y*columns + x + 1;
	}
	
	public static void updateAdjList(Graph g){
		Integer[] vIds = g.adjacencySetMap.keySet().toArray(new Integer[0]);
		for(int i = 0; i < vIds.length; i ++){
			g.adjacencySetMap.get(vIds[i]).remove(vIds[i]);
		}
	}
	
	/**
	 * Is the graph still connected when we remove a vertex?
	 * @param vId
	 * @return
	 */
	public boolean isConnectedWithoutVertex(int vId){
		// Get the neighbors
		Set<Integer> neighborSet = this.adjacencySetMap.get(vId);
		neighborSet.remove(vId);
		
		// If a single neighbor, must be connected
		if(neighborSet.size() == 1) return true;
		
		// For the rest cases, do pairwise test
		Integer[] vIds = neighborSet.toArray(new Integer[0]);
		for(int i = 0; i < vIds.length - 1; i ++){
			if(!areVerticesConnectedWithoutV3(vIds[i], vIds[i + 1], vId)){
				neighborSet.add(vId);
				neighborSet.add(vId);
				return false;
			}
		}
		neighborSet.add(vId);
		return true;
	}
	
	/**
	 * Are vertices with vId1 and vId2 connected without going through vId3?
	 * @param vId1
	 * @param vId2
	 * @param vId3
	 * @return
	 */
	private boolean areVerticesConnectedWithoutV3(int vId1, int vId2, int vId3){
		// House keeping
		Set<Integer> visitedVertices = new HashSet<Integer>();
		List<Integer> queue = new LinkedList<Integer>();
		
		// Add root
		queue.add(vId1);
		
		// Mark vId3 as visited; this automatically skips vId3 during search
		visitedVertices.add(vId3);
		
		// BFS graph to check connectivity of v1 and v2
		while(!queue.isEmpty()){
			int vId = queue.remove(0);
			Set<Integer> is = adjacencySetMap.get(vId);
			if(is != null){
				Integer[] neighborIds = is.toArray(new Integer[0]);
				for(int i = 0; i <neighborIds.length; i++){
					if(!visitedVertices.contains(neighborIds[i])){
						if(neighborIds[i] == vId2){
							return true;
						}
						queue.add(neighborIds[i]);
						visitedVertices.add(neighborIds[i]);
					}
				}
			}
		}
		return false;
	}
	
	protected static int SG[][] = {{0, 3}, {3, 0}};
	protected static Graph getv4TestGraph(){
		Graph g = new Graph();
		Vertex v0 = new Vertex(new Point2D.Double(0, 0));
		v0.id = 0; 
		Vertex v1 = new Vertex(new Point2D.Double(0, 1));
		v1.id = 1; 
		Vertex v2 = new Vertex(new Point2D.Double(1, 1));
		v2.id = 2; 
		Vertex v3 = new Vertex(new Point2D.Double(0, 2));
		v3.id = 3; 
		g.addVertex(v0);
		g.addVertex(v1);
		g.addVertex(v2);
		g.addVertex(v3);
	
		Set<Integer> nbrs0 = new TreeSet<Integer>();
		g.adjacencySetMap.put(0, nbrs0);
		nbrs0.add(0);
		nbrs0.add(1);
		Set<Integer> nbrs1 = new TreeSet<Integer>();
		g.adjacencySetMap.put(1, nbrs1);
		nbrs1.add(0);
		nbrs1.add(1);
		nbrs1.add(2);
		nbrs1.add(3);
		Set<Integer> nbrs2 = new TreeSet<Integer>();
		g.adjacencySetMap.put(2, nbrs2);
		nbrs2.add(1);
		nbrs2.add(2);
		Set<Integer> nbrs3 = new TreeSet<Integer>();
		g.adjacencySetMap.put(3, nbrs3);
		nbrs3.add(1);
		nbrs3.add(3);
		
		g.vertices = g.verticeSet.toArray(new Vertex[0]);
		
		return g;
	}

	static int[][] BRIDGE_TEST_VERTICES= new int[][]{
		{0, 0},
		{0, 1},
		{1, 0},
		{1, 1},
		{2, 1},
		{2, 2},
		{3, 0},
		{3, 1},
		{4, 0},
		{4, 1},
		{4, 2},
		{5, 1},
		{5, 2},
		{6, 1}
	};
	public static Graph getTestGraphForBridge(){
		Graph g = new Graph();
		for(int i = 0; i < BRIDGE_TEST_VERTICES.length; i ++)
		{
			g.addVertex(new Vertex(new Point2D.Double(BRIDGE_TEST_VERTICES[i][0], BRIDGE_TEST_VERTICES[i][1]), i));
		}
		g.vertices = g.verticeSet.toArray(new Vertex[0]);

		for(Vertex v: g.vertices){
			Set<Integer> nbrs = new TreeSet<Integer>();
			g.adjacencySetMap.put(v.id, nbrs);
			for(Vertex n: g.vertices){
				if(Math.abs(v.getIX() - n.getIX()) + Math.abs(v.getIY() - n.getIY()) < 2){
					nbrs.add(n.id);
				}
			}
		}
		g.finishBuildingGraph();
		return g;
	}
	
	/**
	 * Adding a vertex with the set of neighbors
	 * @param vId
	 * @param neighbors
	 */
	public void addVertex(int vId, int[] neighbors){
		// Add vertex
		Vertex v = new Vertex(vId);
		verticeSet.add(v);
		idVertexMap.put(v.id, v);
		
		// Add neighbors
		Set<Integer> vSet = new TreeSet<Integer>(); 
		adjacencySetMap.put(vId, vSet);
		vSet.add(vId);
		for(int i = 0; i < neighbors.length; i ++){
			vSet.add(neighbors[i]);
		}
	}
	
	/**
	 * Finish building graphs
	 */
	public void finishBuildingGraph(){
		vertices = idVertexMap.values().toArray(new Vertex[0]); 
	}
	
	/**
	 * Print graph as adjacency list 
	 */
	public void printGraphAsAdjacenyList(){
		for(int i = 0; i < vertices.length; i ++){
			System.out.print(i + ": ");
			Integer[] vIds = adjacencySetMap.get(i).toArray(new Integer[0]);
			for(int av = 0; av < vIds.length; av++){
				if(vIds[av] != i){
					System.out.print(vIds[av] + ", ");
				}
			}
			System.out.println();
		}
	}
	
	public void paint(Graphics2D g2d, int scale){
		Vertex root = this.idVertexMap.values().toArray(new Vertex[0])[0];
		// House keeping stuff
		Set<Vertex> visitedVertices = new HashSet<Vertex>();
		List<Vertex> queue = new LinkedList<Vertex>();
		
		// Add root
		queue.add(root);
		while(!queue.isEmpty()){
			Vertex v = queue.remove(0);
			Set<Integer> is = adjacencySetMap.get(v.id);
			if(is != null){
				Integer[] neighborIds = is.toArray(new Integer[0]);
				for(int i = 0; i <neighborIds.length; i++){
					Vertex n = idVertexMap.get(neighborIds[i]);
					if(!visitedVertices.contains(n)){
						queue.add(n);
						visitedVertices.add(n);
					}
					if(v.id < n.id){
						g2d.setStroke(new BasicStroke(1));
						g2d.setPaint(new java.awt.Color(0xB0B0D0));

						// Draw edges 
						g2d.drawLine(scale * v.getIX(), scale* v.getIY(), scale * n.getIX(), scale* n.getIY());
					}
				}
			}
		}
	}
	
}
