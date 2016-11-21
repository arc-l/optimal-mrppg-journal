package projects.multipath.ILP;

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
public class HeuristicPathPlanner extends PathPlanner{
	
	public static void main(String argv[]){
		HeuristicPathPlanner pp = new HeuristicPathPlanner();
		Problem p = Problem.getLongStraightWithOneGarageProblem();
		printStartAndGoals(p.graph, p.sg[0], p.sg[1]);
		int start[] = p.sg[0].clone();
		int goal[] = p.sg[1].clone();
		int[][] paths = pp.planHeuristicPaths(p.graph, p.sg[0], p.sg[1]);
		if(paths != null){
			printPaths(p.graph, paths);
			if(isPathSetValid(paths, p.graph, start, goal)){
				System.out.println("Path set is valid.");
			}
		}
	}
	
	public static void printPaths(Graph g, int [][] paths){
		for(int a = 0; a < paths.length; a ++){
			System.out.print("Agent " + a + ":");
			for(int t = 0; t < paths[0].length; t ++){
				System.out.print(" " + t + ":");
				g.vertices[paths[a][t]].printVertex();
			}
			System.out.println();
		}
	}
	
	@Override
	public int[][] planHeuristicPaths(Graph graph, int start[], int goal[]){
		distanceToGoals = new int[start.length][graph.vertices.length];
		for(int a = 0; a < goal.length; a ++){
			PathFinder.getDistancesToAllVertices(graph, goal[a], distanceToGoals[a]);
		}
		
		int[][] ps = null; 
		for(int i = 0; i < 3; i ++){
			System.out.println("Attempting with " + (1.5 + (5 + i * 4)*start.length/120.) + " seconds...");
			int s[] = start.clone();
			int g[] = goal.clone();
			ps = planHeuristicPath(graph, s, g, 1.5 + (5 + i * 4)*start.length/120.);
			if(ps != null){
				 int count = numberOfGoalsReached(ps, goal);
				 if(count == goal.length){
					 return ps;
				 }
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
	
	@Override
	public int[][] planHeuristicPath(Graph g, int s[], int goal[], double timeLimit){
		long time = System.currentTimeMillis(); 
		
		MultiagentGraphSolverGurobiTime.bPrintPaths = false;
		MultiagentGraphSolverGurobiTime.bDebugInfo = false;
		int numAgents = s.length;
		int start[] = s.clone();

		// Print start/goal
		// printStartAndGoals(g, start, goal);
		
		Path[] pathArray = new Path[numAgents];
		for(int a = 0; a < numAgents; a ++){
			pathArray[a] = new Path();
			pathArray[a].addVertex(start[a]);
		}
		
		int progressCountStalled = 0;
		while(true){
			int beginLength = pathArray[0].vertexList.size();
			// printStartAndGoals(g, start, goal);
			
			// Grab paths and convert to array
			int[][] paths = parsePaths(PathFinder.findShortestPaths(g, start, goal));
			if(paths == null || paths[0].length == 1) return parsePaths(pathArray);
			
			// Move agents as much as we can
			int ct = 0;
			while(ct < paths[0].length && existsEdgeCollision(paths, ct) == null && existsVertexCollision(paths, ct) == null){
				// No meet or head on, move agents
				ct ++;
				moveAgentsIndependentlyOneStep(pathArray, paths, ct);
				
				// Are we done?
				if(ct == paths[0].length - 1) return parsePaths(pathArray);
			}
			
			// printPaths(g, parsePaths(pathArray));
			
			// Not done, resolve local conflicts. 
			Vector<Problem> subProbVec = new Vector<Problem>();
			
			// Detecting two agents exchanging locations
			Set<Long> eSet = new HashSet<Long>();
			for(int a = 0; a < numAgents; a ++){
				int v1 = paths[a][ct];
				int v2 = paths[a][ct + 1];
				long eId = v1 > v2 ? v1*10000 + v2 : v2*10000 + v1;
				if(eSet.contains(eId)){
					// Create sub problem
					subProbVec.add(createSubProblem(g, paths, ct, new Integer[]{v1, v2}, goal, 2));
//					subProbVec.insertElementAt(createSubProblem(g, paths, ct, new Integer[]{v1, v2}, goal, 2),
//							(int)(subProbVec.size()*Math.random()));
				}
				else{
					eSet.add(eId);
				}
			}
			
			// Detect two conflicting vertices and grow a little distance from the area
			Set<Integer> vSet = new HashSet<Integer>();
			Set<Integer> processedVSet = new HashSet<Integer>();
			for(int a = 0; a < numAgents; a ++){
				int vId = paths[a][ct + 1];
				if(vSet.contains(vId) && !processedVSet.contains(vId)){
					// Get conflicting vIds
					Vector<Integer> cVIdSet = new Vector<Integer>();
					for(int aa = 0; aa < numAgents; aa++){
						if(paths[aa][ct + 1] == vId){
							cVIdSet.add(paths[aa][ct]);
						}
					}
					
					// Create sub problem
					subProbVec.add(createSubProblem(g, paths, ct, cVIdSet.toArray(new Integer[0]), goal, 2));
//					subProbVec.insertElementAt(createSubProblem(g, paths, ct, cVIdSet.toArray(new Integer[0]), goal, 2)
//							,(int)(subProbVec.size()*Math.random()));
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
						int vId = paths[a][ct];
						// Is this agent part of the subproblem?
						if(p.vidNewIdMap.get(vId) != null){
							int newVId = p.vidNewIdMap.get(vId);
							// Yes, need to update path for this agent
							int subIndex = startVIdAgtMap.get(newVId);
							for(int t = 1; t < subPs[0].length; t ++){
								pathArray[a].addVertex(p.newIdVidMap.get(subPs[subIndex][t]));
							}
						}
					}
					// printStartAndGoals(g, start, goal);
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

			// printPaths(g, parsePaths(pathArray));
			
			for(int a = 0; a < numAgents; a++ )
				start[a] = pathArray[a].vertexList.get(pathArray[a].vertexList.size() - 1);

		}
	}
	
	private int[] existsEdgeCollision(int[][] paths, int t){
		int numAgents = paths.length;
		Map<Long, Integer> eAMap = new HashMap<Long, Integer>();

		for(int a = 0; a < numAgents; a ++){
			int v1 = paths[a][t];
			int v2 = paths[a][t+1];
			long eId = v1 > v2 ? v1*10000 + v2 : v2*10000 + v1;
			if(eAMap.get(eId) != null){
				return new int[]{eAMap.get(eId), a};
			}
			else
			{
				eAMap.put(eId, a);
			}
		}
		return null;
	}
		
	private int[] existsVertexCollision(int[][] paths, int t){
		int numAgents = paths.length;
		Map<Integer, Integer> vAMap = new HashMap<Integer, Integer>();
		for(int a = 0; a < numAgents; a ++){
			int vId = paths[a][t+1];
			if(vAMap.get(vId) != null){
				return new int[]{vAMap.get(vId), a};
			}
			else{
				vAMap.put(vId, a);
			}
		}
		return null;
	}
	
	/**
	 * Move agents at most one step 
	 */
	private void moveAgentsIndependentlyOneStep(Path[] pathArray, int[][]paths, int t){
		int numAgent = paths.length;
		for(int a = 0; a < numAgent; a ++){
			pathArray[a].addVertex(paths[a][t]);
		}
	}

	/**
	 * Create a subproblem around the conflicting vertices
	 * @param g
	 * @param paths
	 * @param ct
	 * @param svs
	 * @param goal
	 * @param distance
	 * @return
	 */
	private Problem createSubProblem(Graph g, int[][] paths, int ct, Integer svs[], int goal[], int distance){
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
						if(distance > 2 || Math.random() > 0.75){
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

		unOccupied.addAll(involved);
		
		for(int a = 0; a < numAgents; a ++){
			int vId = paths[a][ct];
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
		
		int sg[][] = new int[2][newStart.size()];
		for(int i = 0; i < newStart.size(); i ++){
			sg[0][i] = newStart.get(i);
			sg[1][i] = newGoal.get(i);
		}
		
		Problem p = new Problem();
		p.graph = subG;
		p.sg = sg;
		p.vidNewIdMap = vidNewIdMap;
		p.newIdVidMap = newIdVidMap;
		
		return p;
	}
	
}
