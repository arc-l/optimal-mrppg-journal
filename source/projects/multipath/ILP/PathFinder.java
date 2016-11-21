package projects.multipath.ILP;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

import projects.multipath.advanced.Graph;
import projects.multipath.advanced.Path;

public class PathFinder {

	public static boolean bHeuristic = false;

	/**
	 * Compute distance of one vertex to all other vertices of the graph
	 * 
	 * @param g
	 * @param start
	 * @return
	 */
	public static int[] getDistancesToAllVertices(Graph g, int start,
			int[] inputDistances) {
		Set<Integer> visitedVertexIds = new HashSet<Integer>();
		List<Integer> queue = new LinkedList<Integer>();
		queue.add(start);
		int[] distances;
		if (inputDistances == null) {
			distances = new int[g.vertices.length];
		} else {
			distances = inputDistances;
		}
		distances[start] = 0;
		while (queue.size() > 0) {
			int vertex = queue.remove(0);
			Integer[] adjacentVertices = g.adjacencySetMap.get(vertex).toArray(
					new Integer[0]);
			for (int i = 0; i < adjacentVertices.length; i++) {
				if (!visitedVertexIds.contains(adjacentVertices[i])) {
					visitedVertexIds.add(adjacentVertices[i]);
					queue.add(adjacentVertices[i]);
					// This vertex has not been visited, set initial distance
					if (adjacentVertices[i] != start)
						distances[adjacentVertices[i]] = distances[vertex] + 1;
				} else {
					// Update distance if shorter
					if (distances[adjacentVertices[i]] > distances[vertex] + 1) {
						distances[adjacentVertices[i]] = distances[vertex] + 1;
					}
				}
			}
		}
		return distances;
	}
	
	protected static Set<Integer> findGoalsInDistanceRange(Graph g, int start, int dMin, int dMax){
		Set<Integer> vSet = new HashSet<Integer>();
		int distance = 1;
		Set<Integer> visitedSet = new HashSet<Integer>();
		Set<Integer> currentSet = new HashSet<Integer>();
		Set<Integer> toBeVisitedSet = new HashSet<Integer>();
		currentSet.add(start);
		visitedSet.add(start);
		while(distance <= dMax && visitedSet.size() < g.vertices.length){
			Integer vIds[] = currentSet.toArray(new Integer[0]);
			for(int i = 0; i < vIds.length; i ++){
				int vId = vIds[i];
				Set<Integer> nbrs = g.adjacencySetMap.get(vId);
				if (nbrs != null) {
					Integer[] neighborIds = nbrs.toArray(new Integer[0]);
					for (int n = 0; n < neighborIds.length; n++) {
						if (neighborIds[n] != vId) {
							if(!visitedSet.contains(neighborIds[n])){
								toBeVisitedSet.add(neighborIds[n]);
								if(distance >= dMin){
									vSet.add(neighborIds[n]);
								}
							}
						}
					}
				}
			}
			distance ++;
			currentSet = toBeVisitedSet; 
			toBeVisitedSet =  new HashSet<Integer>();
			visitedSet.addAll(currentSet);
		}
		return vSet;
	}

	/**
	 * A* search for all naive paths
	 * 
	 * @param g
	 * @param start
	 * @param goal
	 * @return
	 */
	protected static Path[] findShortestPaths(Graph g, int[] start, int[] goal) {
//		for (Vertex v : g.vertices) {
//			v.usedInPath = 0;
//		}
		Path[] paths = new Path[start.length];
		for (int i = 0; i < paths.length; i++) {
			paths[i] = findOneShortestPath(g, start[i], goal[i], null);
		}
		return paths;
	}

	/**
	 * A* search
	 * 
	 * @param g
	 * @param start
	 * @param goal
	 * @return
	 */
	protected static Path findOneShortestPath(Graph g, int start, int goal,
			Set<Integer> alreadyVisitedVertices) {
		// Check whether start == goal
		if (start == goal) {
			Path p = new Path();
			p.addVertex(start);
			return p;
		}

		// House keeping
		Set<Integer> visitedVertices = null;
		if (alreadyVisitedVertices != null) {
			visitedVertices = alreadyVisitedVertices;
		} else {
			visitedVertices = new HashSet<Integer>();
		}
		SortedMap<Integer, List<Integer>> queueMap = new TreeMap<Integer, List<Integer>>();
		Map<Integer, Integer> parentMap = new HashMap<Integer, Integer>();

		// Add root
		addToQueue(queueMap, g, start, goal);

		boolean pathFound = false;
		// BFS graph using heuristic
		while (!queueMap.isEmpty()) {
			int vId = deQueue(queueMap);
			Set<Integer> is = g.adjacencySetMap.get(vId);
			if (is != null) {
//				SortedMap<Integer, Integer> nMap = new TreeMap<Integer, Integer>();
//				for (int nVId : is) {
//					if (nVId != vId) {
//						nMap.put(g.vertices[nVId].usedInPath*1000000 + (int)(Math.random()*10)*10000 + nVId, nVId);
//					}
//				}
//
//				Integer[] nVIds = nMap.values().toArray(new Integer[0]);
//				for(int nVId: nVIds){
//					if (!visitedVertices.contains(nVId)) {
//						parentMap.put(nVId, vId);
//						if (nVId == goal) {
//							queueMap.clear();
//							pathFound = true;
//							break;
//						}
//						addToQueue(queueMap, g, nVId, goal);
//						visitedVertices.add(nVId);
//					}
//				}
				Integer[] neighborIds = is.toArray(new Integer[0]);
				Vector<Integer> nVec = new Vector<Integer>();
				for (int i = 0; i < neighborIds.length; i++) {
					if (neighborIds[i] != vId) {
						nVec.add(neighborIds[i]);
					}
				}
				while (nVec.size() > 0) {
					int index = (int) (nVec.size() * Math.random());
					int nid = nVec.get(index);
					nVec.remove(index);
					if (!visitedVertices.contains(nid)) {
						parentMap.put(nid, vId);
						if (nid == goal) {
							queueMap.clear();
							pathFound = true;
							break;
						}
						addToQueue(queueMap, g, nid, goal);
						visitedVertices.add(nid);
					}
				}
			}
		}

		if (pathFound) {
			// Recover path
			Path p = new Path();
			p.insertVertex(goal);
//			g.vertices[goal].usedInPath ++;
			int lastVertex = goal;
			while (true) {
				Integer parent = parentMap.get(lastVertex);
				if (parent != null) {
					p.insertVertex(parent);
//					g.vertices[parent].usedInPath ++;
					lastVertex = parent;
					if (lastVertex == start) {
						break;
					}
				}
			}
			return p;
		}

//		if (pathFound) {
//			// Recover path
//			Path p = new Path();
//			p.insertVertex(goal);
//			int lastVertex = goal;
//			while (true) {
//				Integer parent = parentMap.get(lastVertex);
//				if (parent != null) {
//					p.insertVertex(parent);
//					lastVertex = parent;
//					if (lastVertex == start) {
//						break;
//					}
//				}
//			}
//			return p;
//		}
		return null;
	}

	/**
	 * Add to queue
	 * 
	 * @param queueMap
	 * @param g
	 * @param vId
	 * @param target
	 */
	private static void addToQueue(SortedMap<Integer, List<Integer>> queueMap,
			Graph g, int vId, int target) {
		int dist = 0; // g.getHeuristicDist(vId, target);
		List<Integer> list = queueMap.get(dist);
		if (list == null) {
			list = new LinkedList<Integer>();
			queueMap.put(dist, list);
		}
//		list.add(vId);
		if (!bHeuristic) {
			list.add(vId);
		} else {
			list.add((int) (Math.random() * list.size()), vId);
		}
	}

	/**
	 * Dequeue
	 * 
	 * @param queueMap
	 * @return
	 */
	private static int deQueue(SortedMap<Integer, List<Integer>> queueMap) {
		int minKey = queueMap.firstKey();
		List<Integer> list = queueMap.get(queueMap.firstKey());
		int ret = list.remove(0);
		if (list.size() == 0) {
			queueMap.remove(minKey);
		}
		return ret;
	}

	/**
	 * 
	 * @param paths
	 * @return
	 */
	public static int getTotalDistance(int[][] paths){
		int dist = 0;
		for(int a = 0; a < paths.length; a++){
			for(int i = 0; i < paths[a].length - 1; i ++){
				if(paths[a][i] != paths[a][i+1]){
					dist++;
				}
			}
		}
		return dist;
	}

	/**
	 * 
	 * @param hp
	 * @return
	 */
	public static int getTotalDistance(Path[] hp){
		int dist = 0;
		for(int a = 0; a < hp.length; a++){
			dist += (hp[a].vertexList.size() - 1);
		}
		return dist;
	}
	
	public static int getMakespanLowerBound(Graph g, int start[], int goal[]){
		Path[] hp = PathFinder.findShortestPaths(g, start, goal);
		int makespanLb = 0;
		for(int i = 0; i < hp.length; i ++){
			if(hp[i].vertexList.size() > makespanLb){
				makespanLb = hp[i].vertexList.size();
			}
		}
		return makespanLb;
		
	}

	public static int getTotalTimeLowerBound(Graph g, int start[], int goal[]){
		Path[] hp = PathFinder.findShortestPaths(g, start, goal);
		int ttLB = 0;
		for(int i = 0; i < hp.length; i ++){
			ttLB += (hp[i].vertexList.size() - 1);
		}
		return ttLB;
		
	}
}

