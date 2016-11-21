package projects.multipath.ILP;

import gurobi.GRBException;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

import projects.multipath.advanced.Graph;
import projects.multipath.advanced.Path;
import projects.multipath.advanced.Problem;
import projects.multipath.advanced.Vertex;

/**
 * Note that there is a 10000 vertex limitation in the current implementation. 
 * @author Jingjin Yu
 *
 */
public class PathPlanner {
	
	public static void main(String argv[]){
		PathPlanner pp = new PathPlanner();
		Problem p = Problem.getLongStraightWithOneGarageProblem();
		PathPlanner.printStartAndGoals(p.graph, p.sg[0], p.sg[1]);
		int start[] = p.sg[0].clone();
		int goal[] = p.sg[1].clone();
		int[][] paths = pp.planHeuristicPaths(p.graph, p.sg[0], p.sg[1]);
		if(paths != null){
			printPaths(p.graph, paths);
			if(PathPlanner.isPathSetValid(paths, p.graph, start, goal)){
				System.out.println("Path set is valid.");
			}
		}
	}
	
	public static void printPaths(Graph g, int [][] paths){
		for(int a = 0; a < paths.length; a ++){
			System.out.print("Agent " + a + ":");
			for(int t = 0; t < paths[0].length; t ++){
				System.out.print(" " + t + ":");
				// System.out.print(",");
				g.vertices[paths[a][t]].printVertex();
			}
			System.out.println();
		}
	}
	
	public static String printPathToString(Graph g, int [][] paths){
		StringBuffer buffer = new StringBuffer();
		for(int a = 0; a < paths.length; a ++){
			buffer.append("Agent " + (a < 9? " " : "") + (a+1) + ":");
			for(int t = 0; t < paths[0].length; t ++){
				buffer.append(" " + t + ":");
				g.vertices[paths[a][t]].printVertex(buffer);
			}
			buffer.append("\n");
		}
		return buffer.toString();
	}
	
	protected int[][] distanceToGoals = null;
	boolean secondStrategy = false;
	int attempt = 0;
	public int[][] planHeuristicPaths(Graph graph, int start[], int goal[]){
		distanceToGoals = new int[start.length][graph.vertices.length];
		for(int a = 0; a < goal.length; a ++){
			PathFinder.getDistancesToAllVertices(graph, goal[a], distanceToGoals[a]);
		}
		
		int[][] ps = null; 
		secondStrategy = false;
		int totalAttempts = 0;
		double attemptTimeOut = 0, increment = 0;
		switch(start.length){
		case 25: totalAttempts = 8; attemptTimeOut = 1.; increment = 0.1; break; 
		case 50: totalAttempts = 7; attemptTimeOut = 1.5; increment = 0.2; break; 
		case 75: totalAttempts = 6; attemptTimeOut = 2.; increment = 0.25; break; 
		case 100: totalAttempts = 5; attemptTimeOut = 4.; increment = 0.5; break; 
		case 125: totalAttempts = 4; attemptTimeOut = 6; increment = 0.5; break; 
		case 150: totalAttempts = 4; attemptTimeOut = 8.; increment = 0.; break;
		default: totalAttempts = 10; attemptTimeOut = 3; increment = 0; break;
		}
		for(attempt = 1; attempt <= totalAttempts; attempt ++){
			System.out.println("Attempting with " + (attemptTimeOut + attempt*increment) + " seconds...");
			int s[] = start.clone();
			int g[] = goal.clone();
			ps = planHeuristicPath(graph, s, g, attemptTimeOut + attempt*increment);
			if(ps != null){
				 int count = numberOfGoalsReached(ps, goal);
				 if(count == goal.length){
					 return ps;
				 }
			}
			if(secondStrategy = false){
				secondStrategy = true;
			}
		}
		return ps;
	}

	public static int numberOfGoalsReached(int[][] paths, int goal[]){
		int count = 0;
		for(int i = 0; i < goal.length; i ++){
			if(paths[i][paths[i].length-1] == goal[i]){
				count ++;
			}
		}
		return count;
	}
	
	public int[][] planHeuristicPath(Graph g, int start[], int goal[], double timeLimit){
		long time = System.currentTimeMillis(); 
		
		MultiagentGraphSolverGurobiTime.bPrintPaths = false;
		MultiagentGraphSolverGurobiTime.bDebugInfo = false;
		int numAgents = start.length;

		// Print start/goal
		// PathPlanner.printStartAndGoals(g, start, goal);
		
		Path[] pathArray = new Path[numAgents];
		for(int a = 0; a < numAgents; a ++){
			pathArray[a] = new Path();
			pathArray[a].addVertex(start[a]);
		}
		
		int progressCountStalled = 0;
		while(true){
			int beginLength = pathArray[0].vertexList.size();
			// Grab paths and convert to array
			// PathPlanner.printStartAndGoals(g, start, goal);
			
			// Move agents as much as we can
			while(moveAgentsIndependentlyWhenCan(g, start, goal, pathArray));
			// moveAgentsIndependentlyWhenCan(g, start, goal, pathArray);
			
			// PathPlanner.printPaths(g, parsePaths(pathArray));
			
			// Are we done?
			int[][] paths = parsePaths(PathFinder.findShortestPaths(g, start, goal));
			if(paths[0].length == 1){
				break;
			}
			
			// Not done, resolve local conflicts. 
			Vector<Problem> subProbVec = new Vector<Problem>();
			
			// Detecting two agents exchanging locations
			Set<Long> eSet = new HashSet<Long>();
			for(int a = 0; a < numAgents; a ++){
				int v1 = paths[a][0];
				int v2 = paths[a][1];
				long eId = v1 > v2 ? v1*10000 + v2 : v2*10000 + v1;
				if(eSet.contains(eId)){
					// Create sub problem
					if(attempt != 3 && goal.length > 100){
						subProbVec.add(createSubProblem(g, paths, new Integer[]{v1, v2}, goal, 2));
					}
					else
					{
						subProbVec.insertElementAt(createSubProblem(g, paths, new Integer[]{v1, v2}, goal, 2),
							subProbVec.size());
					}
				}
				else{
					eSet.add(eId);
				}
			}
			
			// Detect two conflicting vertices and grow a little distance from the area
			Set<Integer> vSet = new HashSet<Integer>();
			Set<Integer> processedVSet = new HashSet<Integer>();
			for(int a = 0; a < numAgents; a ++){
				int vId = paths[a][1];
				if(vSet.contains(vId) && !processedVSet.contains(vId)){
					// Get conflicting vIds
					Vector<Integer> cVIdSet = new Vector<Integer>();
					for(int aa = 0; aa < numAgents; aa++){
						if(paths[aa][1] == vId){
							cVIdSet.add(paths[aa][0]);
						}
					}
					
					// Create sub problem
					if(attempt != 3 && goal.length > 100)
					{
						subProbVec.add(createSubProblem(g, paths, cVIdSet.toArray(new Integer[0]), goal, 2));
					}
					else{
						subProbVec.insertElementAt(createSubProblem(g, paths, cVIdSet.toArray(new Integer[0]), goal, 2)
								,subProbVec.size());
					}
					processedVSet.add(vId);
				}
				else{
					vSet.add(vId);
				}
			}
			
			// Merge subproblems
			// subProbVec = mergeSubProblems(subProbVec);
			
			// Solve problems that do not interset with previously solved problems
			boolean progress = false;
			int toAdd = 0;
			int startLength = pathArray[0].vertexList.size();
			Set<Integer> usedVertexSet = new HashSet<Integer>();
			for(int pi = 0; pi < subProbVec.size(); pi ++){
				// Do we still have time?
				double timeLeft = timeLimit - (System.currentTimeMillis() - time)/1000.;
				if(timeLeft < 0 ){
					return parsePaths(pathArray);
				}

				Problem p = subProbVec.get(pi);
				// Check whether vertices are already used
				boolean newProb = true;
				for(int i = 0; i < p.graph.vertices.length; i ++){
					if(usedVertexSet.contains(p.newIdVidMap.get(p.graph.vertices[i].id))){
						newProb = false;
						break;
					}
				}
				if(!newProb)continue;
				
				// Solve 
				int[][] subPs = planPathsAdvanced(p.graph, p.sg[0], p.sg[1], true, 0, true, timeLeft);
				
				// Append to existing path set
				if(subPs != null){
					Map<Integer, Integer> startVIdAgtMap = new HashMap<Integer, Integer>();
					for(int i = 0; i < p.sg[0].length; i ++){
						startVIdAgtMap.put(p.sg[0][i], i);
					}
					
					for(int a = 0; a < numAgents; a++){
						int vId = start[a];
						// Is this agent part of the subproblem?
						if(p.vidNewIdMap.get(vId) != null){
							int newVId = p.vidNewIdMap.get(vId);
							// Yes, need to update path for this agent
							int subIndex = startVIdAgtMap.get(newVId);
							for(int t = 1; t < subPs[0].length; t ++){
								pathArray[a].addVertex(p.newIdVidMap.get(subPs[subIndex][t]));
							}
							start[a] = pathArray[a].vertexList.get(pathArray[a].vertexList.size() - 1);
						}
					}
					// PathPlanner.printStartAndGoals(g, start, goal);
					progress = true;
					
					// Update used vertex set
					for(int i = 0; i < p.graph.vertices.length; i ++){
						usedVertexSet.add(p.newIdVidMap.get(p.graph.vertices[i].id));
					}
					
					// Update maxLength if needed
					if(toAdd < subPs[0].length - 1){
						toAdd = subPs[0].length - 1;
					}
				}
				
			}
			
			// If we make progress, pad all paths to the same length
			if(progress){
				int expLength = startLength + toAdd;
				for(int a = 0; a < numAgents; a ++){
					int length = pathArray[a].vertexList.size();
					if(length < expLength){
						int vId = pathArray[a].vertexList.get(pathArray[a].vertexList.size() - 1);
						for(int i = 0; i < expLength - length; i ++){
							pathArray[a].addVertex(vId);
						}
					}
				}
			}
			
			if(pathArray[0].vertexList.size() == beginLength){
				progressCountStalled ++; 
				if(progressCountStalled == 5){
					return parsePaths(pathArray);
				}
			}
		}
		
		return parsePaths(pathArray);
	}
	
	/**
	 * Create a subproblem around the conflicting vertices
	 * @param g
	 * @param paths
	 * @param svs
	 * @param steps
	 * @param bMeet
	 * @return
	 */
	private Problem createSubProblem(Graph g, int[][] paths, Integer svs[], int goal[], int distance){
		int numAgents = paths.length;
 
		// Gather vertices that are of distance two from conflicting vertices
		Set<Integer> involved = new HashSet<Integer>();
		Set<Integer> unOccupied = new HashSet<Integer>();
		for(int a = 0; a < svs.length; a ++){
			involved.add(svs[a]);
			unOccupied.add(svs[a]);
			
			// Distance 1
			Integer[] adjVs = g.adjacencySetMap.get(svs[a]).toArray(new Integer[0]);
			for(int i = 0; i < adjVs.length; i ++){
				involved.add(adjVs[i]);
				// Distance 2
				if(distance > 1){
					Integer[] adj2Vs = g.adjacencySetMap.get(adjVs[i]).toArray(new Integer[0]);
					for(int j = 0; j < adj2Vs.length; j ++){
						involved.add(adj2Vs[j]);
						// Distance 3
						if(distance > 2 || Math.random() > 0.5){
							Integer[] adj3Vs = g.adjacencySetMap.get(adj2Vs[j]).toArray(new Integer[0]);
							for(int k = 0; k < adj3Vs.length; k ++){
								involved.add(adj3Vs[k]);
								// Distance 4
//								if(distance > 3 || Math.random() > 0.999){
//									Integer[] adj4Vs = g.adjacencySetMap.get(adj3Vs[k]).toArray(new Integer[0]);
//									for(int l = 0; l < adj4Vs.length; l ++){
//										involved.add(adj4Vs[l]);
//									}
//								}
							}
						}
					}
				}
			}
		}
		
		// We have all the vertices, create graph
		Graph subG = new Graph();
		Integer[] vs = involved.toArray(new Integer[0]); 
		Map<Integer, Integer> vidNewIdMap = new HashMap<Integer, Integer>();
		Map<Integer, Integer> newIdVidMap = new HashMap<Integer, Integer>();
		
		// Add vertices
		for(int i = 0; i < vs.length; i ++){
			vidNewIdMap.put(vs[i], i);
			newIdVidMap.put(i, vs[i]);
			Vertex v = new Vertex(i);
			subG.verticeSet.add(v);
			subG.idVertexMap.put(v.id, v);
			subG.adjacencySetMap.put(v.id, new HashSet<Integer>());
			subG.adjacencySetMap.get(v.id).add(v.id);
		}
		
		// Build up the rest of the graph
		for(int i = 0; i < vs.length; i ++){
			Integer adjs[] = g.adjacencySetMap.get(vs[i]).toArray(new Integer[0]);
			for(int nbr = 0; nbr < adjs.length; nbr++){
				Integer idInt = vidNewIdMap.get(adjs[nbr]);
				if(idInt != null){
					subG.adjacencySetMap.get(i).add(idInt.intValue());
				}
			}
		}
		subG.finishBuildingGraph();
		
		// Start and goal locations. First locate all agents starts in subG
		Vector<Integer> newStart = new Vector<Integer>();
		Vector<Integer> newGoal = new Vector<Integer>();

		
		if(attempt%2 == 0){
			unOccupied.addAll(involved);
			for(int a = 0; a < numAgents; a ++){
				int vId = paths[a][0];
				if(vidNewIdMap.get(vId) != null){
					// Start is determined
					newStart.add(vidNewIdMap.get(vId));
					
					// Assign goal 
					SortedMap<Integer,Integer> dVMap = new TreeMap<Integer, Integer>();
					for(Integer vId2: unOccupied){
						dVMap.put(distanceToGoals[a][vId2], vId2);
					}
					int goalId = dVMap.get(dVMap.firstKey());
					newGoal.add(vidNewIdMap.get(goalId));
					unOccupied.remove(goalId);
				}
			}
		}
		else{
			for(int a = 0; a < numAgents; a ++){
				int vId = paths[a][0];
				if(vidNewIdMap.get(vId) != null){
					// Start is determined
					newStart.add(vidNewIdMap.get(vId));
					
					// Assign goals 
					if(unOccupied.contains(vId)){
						for(int i = 0; i < svs.length; i ++){
							if(vId == svs[i]){
								if(i == svs.length - 1){
									newGoal.add(vidNewIdMap.get(svs[0]));
								}
								else{
									newGoal.add(vidNewIdMap.get(svs[i + 1]));
								}
							}
						}
					}
					else{
						newGoal.add(vidNewIdMap.get(vId));
					}
				}
			}
		}
		
		int sg[][] = new int[2][newStart.size()];
		for(int i = 0; i < newStart.size(); i ++){
			sg[0][i] = newStart.get(i);
			sg[1][i] = newGoal.get(i);
		}
		
		// Got start and goal, time to plan
		// int[][] tps = planPathsAdvanced(g, sg[0], sg[1], false, -1); 
		
		Problem p = new Problem();
		p.graph = subG;
		p.sg = sg;
		p.vidNewIdMap = vidNewIdMap;
		p.newIdVidMap = newIdVidMap;
		
		return p;
	}
	
	/**
	 * Move agents independents as much as possible. 
	 * @param paths
	 * @return The number of steps that can be moved 
	 */
	protected int moveAgentsIndependently(int[][] paths){
		int stopTime = 1;
		for(;stopTime < paths[0].length; stopTime ++){
			Set<Integer> vSet = new HashSet<Integer>();
			Set<Long> eSet = new HashSet<Long>();
			for(int a = 0; a < paths.length; a ++){
				vSet.add(paths[a][stopTime]);
				long e = paths[a][stopTime] > paths[a][stopTime-1]
						?paths[a][stopTime]*10000+paths[a][stopTime-1]
						:paths[a][stopTime-1]*10000+paths[a][stopTime];
				eSet.add(e);
			}

			// Detect whether agents meet at the same vertex or whether agents cross the same edge
			if(vSet.size() < paths.length || eSet.size() < paths.length){
				break;
			}
		}
		return stopTime - 1;
	}

	/**
	 * Move agents as much as we can 
	 * @param g
	 * @param start
	 * @param goal
	 * @param pathArray
	 */
	private boolean moveAgentsIndependentlyWhenCan(Graph g, int[] start, int[] goal, Path[] pathArray){
		int[][] paths = parsePaths(PathFinder.findShortestPaths(g, start, goal));
		if(paths[0].length == 1) return false;
		
		int length = pathArray[0].vertexList.size();
		
		int numAgent = paths.length;
		int currentStart[] = new int[numAgent];
		for(int a = 0; a < numAgent; a ++){
			currentStart[a] = 0;
		}

		int count = 5;
		while(count > 0){
			count = 0;
			
			Set<Integer> currentlyOccupiedVIds = new HashSet<Integer>();
			Set<Integer> toBeOccupiedVIds = new HashSet<Integer>();
			Set<Integer> toBeMultiplyOccupiedVIds = new HashSet<Integer>();
			int nextVs[] = new int[numAgent];
			for(int a = 0; a < numAgent; a ++){
				currentlyOccupiedVIds.add(paths[a][currentStart[a]]);
				nextVs[a] = (currentStart[a] < paths[a].length - 1)
						?paths[a][currentStart[a] + 1]:paths[a][currentStart[a]];
				toBeOccupiedVIds.add(nextVs[a]);
			}

			for(int a = 0; a < numAgent; a ++){
				if(toBeOccupiedVIds.contains(nextVs[a])){
					toBeOccupiedVIds.remove(nextVs[a]);
				}
				else{
					toBeMultiplyOccupiedVIds.add(nextVs[a]);
				}
			}

			boolean movable = false;
			for(int a = 0; a < numAgent; a ++){
				if(currentStart[a] < paths[a].length - 1){
					if(!currentlyOccupiedVIds.contains(nextVs[a]))// && !toBeMultiplyOccupiedVIds.contains(nextVs[a]))
 					{
						movable = true;
						break;
					}
				}
			}
			if(!movable)break;

			// Move while we can
			for(int a = 0; a < numAgent && movable; a ++){
				if(currentStart[a] < paths[a].length - 1){
					int nextV = paths[a][currentStart[a] + 1];
					if(!currentlyOccupiedVIds.contains(nextV))// && !toBeMultiplyOccupiedVIds.contains(nextVs[a]))
					{
						pathArray[a].addVertex(nextV);
						currentlyOccupiedVIds.add(nextV);
						currentStart[a]++;
						count ++;
					}
					else{
						pathArray[a].addVertex(paths[a][currentStart[a]]);
					}
				}
				else{
					pathArray[a].addVertex(paths[a][currentStart[a]]);
				}
			}
		}
		
		// Update start vertices
		for(int a = 0; a < numAgent; a ++){
			start[a] = paths[a][currentStart[a]];
		}
		
		return (pathArray[0].vertexList.size() - length > 0);
	}
	
	/**
	 * Convert paths to integer arrays
	 * @param paths
	 * @return
	 */
	protected static int[][] parsePaths(Path[] paths){
		int maxLength = 0;
		for(int i = 0; i < paths.length; i ++){
			if(maxLength < paths[i].vertexList.size()){
				maxLength = paths[i].vertexList.size();
			}
		}
		maxLength--;
		
		int[][] ps = new int[paths.length][maxLength+1];
		for(int i = 0; i < paths.length; i ++){
			for(int t = 0; t <= maxLength; t++){
				if(paths[i].vertexList.size() > t){
					ps[i][t] = paths[i].vertexList.get(t);
				}
				else{
					ps[i][t] = ps[i][t-1];
				}
			}
		}
		return ps;
	}
	
	private int[] splitPaths(Graph g, int start[], int goal[], boolean random){
		int middle[] = new int[start.length]; 
		Path paths[] = PathFinder.findShortestPaths(g, start, goal);

		// Sort all paths by their lengths in descending order - long paths should be processed first
		SortedMap<Integer, Vector<Integer>> lengthIdMap = new TreeMap<Integer, Vector<Integer>>();
		for(int i = 0; i < paths.length; i ++){
			Vector<Integer> indexSet = lengthIdMap.get(-paths[i].vertexList.size());
			if(indexSet == null){
				indexSet = new Vector<Integer>();
				lengthIdMap.put(-paths[i].vertexList.size(), indexSet);
			}
			indexSet.add(i);
		}
		Integer[] keyArray = lengthIdMap.keySet().toArray(new Integer[0]);
		int[] vArray = new int[paths.length];
		int vIndex = 0;
		for(int i = 0; i < keyArray.length; i ++){
			Integer[] indexArray = lengthIdMap.get(keyArray[i]).toArray(new Integer[0]);
			for(int in = 0; in < indexArray.length; in ++){
				vArray[vIndex++] = indexArray[in];
			}
		}
		
		if(random){
			Vector<Integer> vec = new Vector<Integer>();
			for(int i = 0; i < vArray.length; i++){
				vec.add(i);
			}
			for(int i = 0; i < vArray.length; i++){vArray[i] = vec.remove((int)(vec.size()*Math.random()));}
		}
		
		Set<Integer> usedVertexSet = new HashSet<Integer>();
		for(int i = 0; i < paths.length; i ++){
			// Find an intermediate goal that's about halfway
			int dMin = paths[vArray[i]].vertexList.size()/2;
			int dMax = paths[vArray[i]].vertexList.size()/2; 
			boolean foundVertex = false;
			boolean flipped = true;
			while(!foundVertex){
				Set<Integer> fromStartSet = PathFinder.findGoalsInDistanceRange(g, start[vArray[i]], dMin, dMax);
				Set<Integer> fromGoalSet = PathFinder.findGoalsInDistanceRange(g, goal[vArray[i]], dMin, dMax);
				
				// Find intersection
				Set<Integer> iSet = new HashSet<Integer>();
				for(int v: fromStartSet){
					if(fromGoalSet.contains(v)){
						iSet.add(v);
					}
				}
				
				// Need some vertex not in usedVertexSet
				for(int v: iSet){
					if(!usedVertexSet.contains(v)){
						middle[vArray[i]] = v;
						usedVertexSet.add(v);
						foundVertex = true;
						break;
					}
				}
				
				if(!foundVertex){
					if(flipped){
						dMin -= 1;
						if(dMin < 0){dMin = 0;}
					}
					else{
						dMax += 1;
					}
					flipped = !flipped;
				}
			}
		}
		return middle;
	}
	
	protected int[][] planPathsAdvancedSplit(Graph g, int start[], int goal[], boolean setOptimal, double timeLimit, int splits) {
		if(splits > 0){
			// Split the problem into two halves
			int middle[] = splitPaths(g, start, goal,false); 
			int[][] pathFromStart = planPathsAdvancedSplit(g, start, middle, setOptimal, timeLimit/2, splits-1);
			int[][] pathFromMiddle = planPathsAdvancedSplit(g, middle, goal, setOptimal, timeLimit/2, splits-1);
			if(pathFromStart != null && pathFromMiddle != null){
				int[][] fullPaths = new int[pathFromStart.length][pathFromStart[0].length + pathFromMiddle[0].length - 1];
				for(int i = 0; i < pathFromStart.length; i ++){
					for(int j = 0; j < pathFromStart[0].length; j++){
						fullPaths[i][j] = pathFromStart[i][j]; 
					}
					for(int j = 1; j < pathFromMiddle[0].length; j++){
						fullPaths[i][pathFromStart[0].length + j - 1] = pathFromMiddle[i][j]; 
					}
				}
				return fullPaths;
			}
		}
		else{
			return planPathsAdvanced(g, start, goal, false, 0, setOptimal, timeLimit);
		}

		return null;
	}


	protected int[][] planPathsAdvancedSplitDist(Graph g, int start[], int goal[], double gap, double timeLimit, int splits) {
		if(splits > 0){
			// Split the problem into two halves
			int middle[] = splitPaths(g, start, goal,false); 
			double beginTime = System.currentTimeMillis()/1000.;
			int[][] pathFromStart = planPathsAdvancedSplitDist(g, start, middle, gap,  MultiagentGraphSolverGurobiDistance.bReturnAnySolution ? timeLimit/2 : timeLimit, splits-1);
			double elapsedTime  = System.currentTimeMillis()/1000. - beginTime;
			timeLimit -= elapsedTime;
			if(timeLimit > 0){
				beginTime = System.currentTimeMillis()/1000.;
				int[][] pathFromMiddle = planPathsAdvancedSplitDist(g, middle, goal, gap,  timeLimit, splits-1);
				elapsedTime  = System.currentTimeMillis()/1000. - beginTime;
				timeLimit -= elapsedTime;
				if(pathFromStart != null && pathFromMiddle != null){
					int[][] fullPaths = new int[pathFromStart.length][pathFromStart[0].length + pathFromMiddle[0].length - 1];
					for(int i = 0; i < pathFromStart.length; i ++){
						for(int j = 0; j < pathFromStart[0].length; j++){
							fullPaths[i][j] = pathFromStart[i][j]; 
						}
						for(int j = 1; j < pathFromMiddle[0].length; j++){
							fullPaths[i][pathFromStart[0].length + j - 1] = pathFromMiddle[i][j]; 
						}
					}
					return fullPaths;
				}
			}
		}
		else{
			int[][] tp = planPathsAdvanced(g, start, goal, false, 0, false, timeLimit);
			return planPathsAdvancedTD(g, start, goal, gap, tp[0].length - 1, timeLimit);
		}

		return null;
	}


	protected int[][] planPathsAdvancedSplitNoCycle(Graph g, int start[], int goal[], boolean setOptimal, double timeLimit, int splits, boolean top) {
		boolean printPaths = false;
		if(MultiagentGraphSolverGurobiTime.bPrintPaths == true && top){
			MultiagentGraphSolverGurobiTime.bPrintPaths = false;
			printPaths = true;
		}
		int tries = (int)Math.pow(3, 4-splits);
		if(splits > 0){
			// Split the problem into two halves
			int middle[] = splitPaths(g, start, goal,false); 
			int[][] pathFromStart = planPathsAdvancedSplitNoCycle(g, start, middle, setOptimal, timeLimit/2, splits-1, false);
			int[][] pathFromMiddle = planPathsAdvancedSplitNoCycle(g, middle, goal, setOptimal, timeLimit/2, splits-1, false);
			if(pathFromStart == null || pathFromMiddle == null){return null;}
			if(splits == 1){
				while((hasCycles(pathFromStart)>0 || hasCycles(pathFromMiddle)>0 ) && (tries-- > 0 || top) ){
					middle = splitPaths(g, start, goal,true); 
					pathFromStart = planPathsAdvancedSplitNoCycle(g, start, middle, setOptimal, timeLimit/2, splits-1, false);
					pathFromMiddle = planPathsAdvancedSplitNoCycle(g, middle, goal, setOptimal, timeLimit/2, splits-1, false);			
				}
			}
			else{
				while((hasCycles(pathFromStart)>0) && (tries-- > 0 || top) ){
					pathFromStart = planPathsAdvancedSplitNoCycle(g, start, middle, setOptimal, timeLimit/2, splits-1, false);
				}
				while((hasCycles(pathFromMiddle)>0) && (tries-- > 0 || top) ){
					pathFromMiddle = planPathsAdvancedSplitNoCycle(g, middle, goal, setOptimal, timeLimit/2, splits-1, false);			
				}
			}
//			while(pathFromStart != null && pathFromMiddle != null && (hasCycles(pathFromStart)>0 || hasCycles(pathFromMiddle)>0) && (tries-- > 0 || top)){
//				middle = splitPaths(g, start, goal,true); 
//				pathFromStart = planPathsAdvancedSplitNoCycle(g, start, middle, setOptimal, timeLimit/2, splits-1, false);
//				pathFromMiddle = planPathsAdvancedSplitNoCycle(g, middle, goal, setOptimal, timeLimit/2, splits-1, false);			
//			}
			if(pathFromStart != null && pathFromMiddle != null){
				int[][] fullPaths = new int[pathFromStart.length][pathFromStart[0].length + pathFromMiddle[0].length - 1];
				for(int i = 0; i < pathFromStart.length; i ++){
					for(int j = 0; j < pathFromStart[0].length; j++){
						fullPaths[i][j] = pathFromStart[i][j]; 
					}
					for(int j = 1; j < pathFromMiddle[0].length; j++){
						fullPaths[i][pathFromStart[0].length + j - 1] = pathFromMiddle[i][j]; 
					}
				}
				if(printPaths == true && top){
					printPaths(g, fullPaths);
					MultiagentGraphSolverGurobiTime.bPrintPaths = true;
				}
				return fullPaths;
			}
		}
		else{
			return planPathsAdvanced(g, start, goal, false, 0, setOptimal, timeLimit);
		}

		if(printPaths == true && top){
			MultiagentGraphSolverGurobiTime.bPrintPaths = true;
		}
		return null;
	}


	private int getLongestPathLength(Graph g, int start[], int goal[]){
		Path paths[] = PathFinder.findShortestPaths(g, start, goal);
		int length = 0;
		for(int i = 0; i < paths.length; i ++){
			if(paths[i].vertexList.size() - 1 > length){
				length = paths[i].vertexList.size();
			}
		}
		return length;
	}

	private int[] splitPathBySize(Graph g, int start[], int goal[], boolean random, int firstSize){
		int middle[] = new int[start.length]; 
		Path paths[] = PathFinder.findShortestPaths(g, start, goal);

		// Sort all paths by their lengths in descending order - long paths should be processed first
		SortedMap<Integer, Vector<Integer>> lengthIdMap = new TreeMap<Integer, Vector<Integer>>();
		for(int i = 0; i < paths.length; i ++){
			Vector<Integer> indexSet = lengthIdMap.get(-paths[i].vertexList.size());
			if(indexSet == null){
				indexSet = new Vector<Integer>();
				lengthIdMap.put(-paths[i].vertexList.size(), indexSet);
			}
			indexSet.add(i);
		}
		Integer[] keyArray = lengthIdMap.keySet().toArray(new Integer[0]);
		int[] vArray = new int[paths.length];
		int vIndex = 0;
		for(int i = 0; i < keyArray.length; i ++){
			Integer[] indexArray = lengthIdMap.get(keyArray[i]).toArray(new Integer[0]);
			for(int in = 0; in < indexArray.length; in ++){
				vArray[vIndex++] = indexArray[in];
			}
		}
		
		if(random){
			Vector<Integer> vec = new Vector<Integer>();
			for(int i = 0; i < vArray.length; i++){
				vec.add(i);
			}
			for(int i = 0; i < vArray.length; i++){vArray[i] = vec.remove((int)(vec.size()*Math.random()));}
		}
		
		Set<Integer> usedVertexSet = new HashSet<Integer>();
		for(int i = 0; i < paths.length; i ++){
			// Find an intermediate goal that's about halfway
			int dMin = firstSize - 1;
			int dMax = firstSize - 1;
			int dMinGoal = paths[vArray[i]].vertexList.size() - dMax; 
			int dMaxGoal = paths[vArray[i]].vertexList.size() - dMin; 
			boolean foundVertex = false;
			while(!foundVertex){
				Set<Integer> fromStartSet = PathFinder.findGoalsInDistanceRange(g, start[vArray[i]], dMin, dMax);
				Set<Integer> fromGoalSet = PathFinder.findGoalsInDistanceRange(g, goal[vArray[i]], dMinGoal, dMaxGoal);
				
				// Find intersection
				Set<Integer> iSet = new HashSet<Integer>();
				for(int v: fromStartSet){
					if(fromGoalSet.contains(v)){
						iSet.add(v);
					}
				}
				
				// Need some vertex not in usedVertexSet
				for(int v: iSet){
					if(!usedVertexSet.contains(v)){
						middle[vArray[i]] = v;
						usedVertexSet.add(v);
						foundVertex = true;
						break;
					}
				}
				
				if(!foundVertex){
					dMin -= 1;
					if(dMin < 0){dMin = 0;}
					dMaxGoal = paths[vArray[i]].vertexList.size() - dMin;
				}
			}
		}
		return middle;
	}
	
	
	protected int[][] planPathsAdvancedSplitBySizeNoCycle(Graph g, int start[], int goal[], boolean setOptimal, double timeLimit, int size, boolean top) {
		int maxLength = getLongestPathLength(g, start, goal);
		int tries = 5; 
		if(maxLength > size){
			// Split the problem into two halves
			int middle[] = splitPathBySize(g, start, goal,false,size); 
			int[][] pathFromStart = planPathsAdvancedSplitBySizeNoCycle(g, start, middle, setOptimal, timeLimit/2, size, false);
			int[][] pathFromMiddle = planPathsAdvancedSplitBySizeNoCycle(g, middle, goal, setOptimal, timeLimit/2, size, false);
			while(pathFromStart != null && pathFromMiddle != null && (hasCycles(pathFromStart)>0 || hasCycles(pathFromMiddle)>0) && (tries-- > 0 || top)){
				middle = splitPaths(g, start, goal,true); 
				pathFromStart = planPathsAdvancedSplitBySizeNoCycle(g, start, middle, setOptimal, timeLimit/2, size, false);
				pathFromMiddle = planPathsAdvancedSplitBySizeNoCycle(g, middle, goal, setOptimal, timeLimit/2, size, false);			
			}
			if(pathFromStart != null && pathFromMiddle != null){
				int[][] fullPaths = new int[pathFromStart.length][pathFromStart[0].length + pathFromMiddle[0].length - 1];
				for(int i = 0; i < pathFromStart.length; i ++){
					for(int j = 0; j < pathFromStart[0].length; j++){
						fullPaths[i][j] = pathFromStart[i][j]; 
					}
					for(int j = 1; j < pathFromMiddle[0].length; j++){
						fullPaths[i][pathFromStart[0].length + j - 1] = pathFromMiddle[i][j]; 
					}
				}
				return fullPaths;
			}
		}
		else{
			return planPathsAdvanced(g, start, goal, false, 0, setOptimal, timeLimit);
		}

		return null;
	}

	protected int[][] planPathsAdvancedSplitBySize(Graph g, int start[], int goal[], boolean setOptimal, double timeLimit, int size) {
		int maxLength = getLongestPathLength(g, start, goal);
		if(maxLength > size){
			// Split the problem into two halves
			int middle[] = splitPathBySize(g, start, goal,false,size); 
			int[][] pathFromStart = planPathsAdvancedSplitBySize(g, start, middle, setOptimal, timeLimit/2, size);
			int[][] pathFromMiddle = planPathsAdvancedSplitBySize(g, middle, goal, setOptimal, timeLimit/2, size);
			if(pathFromStart != null && pathFromMiddle != null){
				int[][] fullPaths = new int[pathFromStart.length][pathFromStart[0].length + pathFromMiddle[0].length - 1];
				for(int i = 0; i < pathFromStart.length; i ++){
					for(int j = 0; j < pathFromStart[0].length; j++){
						fullPaths[i][j] = pathFromStart[i][j]; 
					}
					for(int j = 1; j < pathFromMiddle[0].length; j++){
						fullPaths[i][pathFromStart[0].length + j - 1] = pathFromMiddle[i][j]; 
					}
				}
				return fullPaths;
			}
		}
		else{
			return planPathsAdvanced(g, start, goal, false, 0, setOptimal, timeLimit);
		}

		return null;
	}

	private int[][] mergePaths(int[][] pathFromStart, int[][] pathFromMiddle){
		if(pathFromStart != null && pathFromMiddle != null){
			int[][] fullPaths = new int[pathFromStart.length][pathFromStart[0].length + pathFromMiddle[0].length - 1];
			for(int i = 0; i < pathFromStart.length; i ++){
				for(int j = 0; j < pathFromStart[0].length; j++){
					fullPaths[i][j] = pathFromStart[i][j]; 
				}
				for(int j = 1; j < pathFromMiddle[0].length; j++){
					fullPaths[i][pathFromStart[0].length + j - 1] = pathFromMiddle[i][j]; 
				}
			}
			return fullPaths;
		}
		return null;
	}
	
	protected int[][] planPathsAdvanced(Graph g, int start[], int goal[], boolean solveRelaxed, int extraSteps, boolean setOptimal, double timeLimit) {
		MultiagentGraphSolverGurobiTime solver = new MultiagentGraphSolverGurobiTime();
		try {
			int paths[][] =  solver.solve(g, start, goal, solveRelaxed, setOptimal, extraSteps, timeLimit > 0, timeLimit);
			if(paths != null){
				if(!isPathSetValid(paths, g, start, goal)){
					return null;
				}
			}
			else{
				if(MultiagentGraphSolverGurobiTime.bDebugInfo){
					System.out.println("Maximum allowed time reached, optimization stopped.");
				}
			}
			return paths;
		} catch (GRBException e) {
			e.printStackTrace();
		}

		return null;
	}

	protected int[][] planPathsAdvancedPMG(Graph g, int start[], int goal[], boolean solveRelaxed, int extraSteps, boolean setOptimal, double timeLimit) {
		MultiagentGraphSolverGurobiTimeR solver = new MultiagentGraphSolverGurobiTimeR();
		try {
			int paths[][] =  solver.solve(g, start, goal, solveRelaxed, setOptimal, extraSteps, timeLimit > 0 , timeLimit);
			if(paths != null){
				if(!isPathSetValid(paths, g, start, goal)){
					return null;
				}
			}
			else{
				if(MultiagentGraphSolverGurobiTime.bDebugInfo){
					System.out.println("Maximum allowed time reached, optimization stopped.");
				}
			}
			return paths;
		} catch (GRBException e) {
			e.printStackTrace();
		}

		return null;
	}

	/**
	 * Planning total time optimal solution
	 * @param g
	 * @param start
	 * @param goal
	 * @param opt
	 * @param gap
	 * @param timeSteps
	 * @param timeLimit
	 * @return
	 */
	protected int[][] planPathsAdvancedTT(Graph g, int start[], int goal[], int opt[], double gap, int timeSteps, double timeLimit) {
		MultiagentGraphSolverGurobiTotalTime solver = new MultiagentGraphSolverGurobiTotalTime();
		try {
			int paths[][] =  solver.solve(g, start, goal, opt, gap, timeSteps, timeLimit > 0 , timeLimit);
			if(paths != null){
				if(!isPathSetValid(paths, g, start, goal)){
					return null;
				}
			}
			else{
				if(MultiagentGraphSolverGurobiTime.bDebugInfo){
					System.out.println("Maximum allowed time reached, optimization stopped.");
				}
			}
			return paths;
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return null;
	}
	
	protected int[][] planPathsAdvancedTD(Graph g, int start[], int goal[], double gap, int timeSteps, double timeLimit) {
		MultiagentGraphSolverGurobiDistance solver = new MultiagentGraphSolverGurobiDistance();
		try {
			int paths[][] =  solver.solve(g, start, goal, gap, timeSteps, timeLimit);
			if(paths != null){
				if(!isPathSetValid(paths, g, start, goal)){
					return null;
				}
			}
			else{
				if(MultiagentGraphSolverGurobiTime.bDebugInfo){
					System.out.println("Maximum allowed time reached, optimization stopped.");
				}
			}
			return paths;
		} catch (GRBException e) {
			e.printStackTrace();
		}
		return null;
	}

	int mpieces = 0;
	int msg[][] = null;
	boolean mdone[] = null;
	Graph mg = null;
	Map<Integer, Object> mpath = new HashMap<Integer, Object>();
	double mtime = 0;
	protected int[][] multiThreadedSplitPlanning(Graph g, int start[], int goal[], double timeLimit, int splits){
		mg = g;
		mtime = timeLimit;
		mpieces = (int)Math.pow(2, splits);
		msg = new int[mpieces + 1][start.length];
		mdone = new boolean[mpieces];
		for(int i = 0;i < start.length; i ++){
			msg[0][i] = start[i];
			msg[mpieces][i] = goal[i];
		}
		
		boolean printPaths = false;
		if(MultiagentGraphSolverGurobiTime.bPrintPaths == true){
			MultiagentGraphSolverGurobiTime.bPrintPaths = false;
			printPaths = true;
		}

		// Do splitting
		for(int i = 0; i < splits; i ++){
			int numSplits = (int)Math.pow(2, i);
			int stepSize = (int)Math.pow(2, splits - i);
			for(int s = 0; s < numSplits; s++){
				int[] middle = splitPaths(g, msg[s*stepSize], msg[(s+1)*stepSize], false);
				for(int a = 0; a < start.length; a ++){
					msg[s*stepSize + stepSize/2][a] = middle[a];
				}
			}
		}
		
		// Plan each piece using a thread
		for(int i = 0; i < mpieces; i ++){
			Thread x = createThread(i);
			x.start();
		}
		
		// Wait for work to finish
		boolean allDone = false;
		while(!allDone){
			allDone = true;
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			for(int i = 0; i < mpieces; i ++){
				if(mdone[i] == false){
					allDone = false;
					break;
				}
			}
		}
		
		// All done, piece together the paths
		int[][] allPaths = (int[][])(mpath.get(0));
		for(int i = 1;i < mpieces; i ++){
			allPaths = mergePaths(allPaths, (int[][])(mpath.get(i)));
		}
		
		if(printPaths == true){
			MultiagentGraphSolverGurobiTime.bPrintPaths = true;
			if(allPaths != null){printPaths(g, allPaths);}
		}
		
		return allPaths;
	}

	private Thread createThread(final int i){
		return new Thread(
				new Runnable(){
					@Override
					public void run(){
						MultiagentGraphSolverGurobiTime solver = new MultiagentGraphSolverGurobiTime();
						try {
							int paths[][] =  solver.solve(mg, msg[i], msg[i+1], false, true, 0, true , mtime);
							if(paths != null){
								if(isPathSetValid(paths, mg, msg[i], msg[i+1])){
									mpath.put(i, paths);
								}
							}
							else{
								if(MultiagentGraphSolverGurobiTime.bDebugInfo){
									System.out.println("Maximum allowed time reached, optimization stopped.");
								}
							}
							mdone[i] = true;
						} catch (GRBException e) {
							e.printStackTrace();
						}
					};
				});
	}
	
	protected static void printStartAndGoals(Graph g, int start[], int goal[]) {
		System.out.println("Start and goal vertices: " + start.length
				+ " pairs");
		for (int i = 0; i < start.length; i++) {
			Vertex v = g.idVertexMap.get(start[i]);
			v.printVertex();
			System.out.print(" ");
		}
		System.out.println(" ");
		for (int i = 0; i < goal.length; i++) {
			Vertex v = g.idVertexMap.get(goal[i]);
			v.printVertex();
			System.out.print(" ");
		}
		System.out.println(" ");
	}

	public static int hasCycles(int paths[][]){
		int numCycles = 0;
		for(int i = 0;i < paths[0].length - 1; i ++){
			Map<Integer, Integer> fromToMap = new HashMap<Integer, Integer>();
			
			for(int v = 0; v < paths.length; v ++){
				fromToMap.put(paths[v][i], paths[v][i+1]);
			}
			
			for(int s = 0; s < paths.length; s++){
				int vId = paths[s][i];
				if(fromToMap.containsKey(vId)){
					if(fromToMap.get(vId) == vId){
						fromToMap.remove(vId);
						continue;
					}
				}
				else{
					continue;
				}
				while(fromToMap.containsKey(vId)){
					// See whether we have a cycle
					int nextVId = fromToMap.get(vId);
					fromToMap.remove(vId);
					vId = nextVId;
				}
				if(vId == paths[s][i]){
					numCycles++;
				}
			}
		}
		return numCycles;
	}

	public static boolean isPathSetValid(int paths[][], Graph graph,  int starts[], int goals[]){
		// Check start/goal and individual path validity
		for(int t = 0; t < paths[0].length; t ++){
			for(int a = 0; a < paths.length; a++){
				// Check start/goal location
				if(paths[a][0] != starts[a]){
					System.out.println("Start location for agent " + a + " is incorrect in the path set.");
					return false;
				}
				if(paths[a][paths[a].length - 1] != goals[a]){
					System.out.println("Goal location for agent " + a + " is incorrect in the path set.");
					return false;
				}
				
				// Check next vertex is adjacent to the current one
				if(t > 0){
					int lastV = paths[a][t - 1];
					int curV = paths[a][t];
					if(!graph.adjacencySetMap.get(lastV).contains(curV)){
						System.out.println("Path " + a + " is not valid at t = " + (t-1));
						return false;
					}
				}
			}
		}
		
		// Do pair wise meet/head-on collision check
		for(int p = 0; p < paths.length - 1; p ++){
			for(int q = p + 1; q < paths.length; q++){
				for(int t = 0; t < paths[0].length; t ++){
					// Check meet collision
					if(paths[p][t] == paths[q][t]){
						System.out.println("Path " + p + " and " + q + " meet on vertex " + paths[p][t] + " at time step " + t);
						return false;
					}
					// Check head on collision
					if(t > 0 && (paths[p][t] == paths[q][t-1]) && (paths[p][t-1] == paths[q][t])){
						System.out.println("Path " + p + " and " + q + " cross the same edge at time step " + t);
						return false;
					}
				}
			}
		}
		return true;
	}
}
