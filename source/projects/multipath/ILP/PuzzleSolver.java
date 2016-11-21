package projects.multipath.ILP;

import projects.multipath.advanced.Graph;
import projects.multipath.advanced.Path;
import projects.multipath.advanced.Problem;
import gurobi.GRB;
import gurobi.GRBCallback;
import gurobi.GRBException;

public class PuzzleSolver extends GRBCallback{
	
	protected boolean optimizationDone = false;
	protected int goodSolution = 0;

	Graph graph = null;
	int[] starts = null, goals = null;
	PuzzleSolver callback = null;
	int[][] sol1 = null, sol2 = null;
	boolean flag1 = false, flag2 = false;
	private boolean reachableVertices[][][] = null;
	int startTimeSteps = 0;
	boolean failedAttempt = false;
	boolean allThreadFinished = false;
	
	public PuzzleSolver(Graph graph, int starts[], int goals[]){
		this.goals = goals;
		this.graph = graph;
		this.starts = starts;
	}
	
	protected int[][] solve(double timeLimit){
		long time = System.currentTimeMillis();
		long minutes = 0;
		callback = this;

		// Get minimal time steps to start
		// Check minimal distances necessary for solving the problem
		Path paths[] = PathFinder.findShortestPaths(graph, starts, goals);
		for(int i = 0; i < paths.length; i ++){
			if(paths[i].vertexList.size() - 1 > startTimeSteps){
				startTimeSteps = paths[i].vertexList.size() - 1;
			}
		}

		
		while(true){
			updateReachableSets(graph, starts, goals, startTimeSteps);
//			System.out.println("Working with T = " + startTimeSteps);
			Thread firstThread = createThread(1);
			Thread secondThread = createThread(2);
			firstThread.start();
			secondThread.start();
			
			while(!optimizationDone){
				try {
					Thread.sleep(1000);
					// Check time limit
					if(timeLimit > 0 && (System.currentTimeMillis() - time)/1000. > timeLimit){
						optimizationDone = true; 
						return null;
					}
					
					if(failedAttempt){
						optimizationDone = true;
						while(!allThreadFinished){
							// Wait for other threads to finish
							try {
								Thread.sleep(100);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
						}
						allThreadFinished = false;
						failedAttempt = false;
						break;
					}
					
					// Print status
					long min = (System.currentTimeMillis() - time)/60000;
					if(min > minutes){
						minutes = min;
						System.out.print(".");
					}
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			
			while(!flag1 || !flag2){
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			flag1 = flag2 = false;
			
			if(sol1 == null && sol2 == null){
				optimizationDone = false;
				startTimeSteps ++;
			}
			else{
				System.out.println();
				int[][] sol = goodSolution == 1? sol1 : sol2;
				return sol;
			}
		}
	}

	private Thread createThread(final int instance){
		return new Thread(
				new Runnable(){
					@Override
					public void run(){
						MultiagentGraphSolverGurobiTimePuzzle solver = new MultiagentGraphSolverGurobiTimePuzzle();
						solver.instance = instance;
						try {
							int[][] sol = solver.solve(graph, starts, goals, instance == 1 ? false : true, callback, startTimeSteps, reachableVertices);
							if(solver.instance == 1){
								sol1 = sol;
								flag1 = true;
							}
							else{
								sol2 = sol;
								flag2 = true;
							}
						} catch (GRBException e) {
							e.printStackTrace();
						}
					};
				});
	}
	
	private void updateReachableSets(Graph graph, int starts[], int goals[], int startTimeSteps){
		/**
		 *  Preparation. First compute whether a vertex is reachable by an agent at a time step.
		 *  For each agent with start x_i and goal x_g, a vertex v at time t is reachable if  
		 *  dist(x_i, v) <=  t and dist(v, x_g) <= T - t.   
		 */
		reachableVertices = new boolean[starts.length][graph.vertices.length][startTimeSteps + 1];
		for(int a = 0; a < starts.length; a ++){
			// Compute the distances from start/goal to all vertices of the graph   
			int[] distFromStart = PathFinder.getDistancesToAllVertices(graph, starts[a], null); 
			int[] distFromGoal = PathFinder.getDistancesToAllVertices(graph, goals[a], null);
			
			// Assign reachability values
			for(int t = 0; t <= startTimeSteps; t ++){
				for(int v = 0; v < graph.vertices.length; v ++){
					if(distFromStart[v] <= t && distFromGoal[v] <= startTimeSteps - t){
						reachableVertices[a][v][t] = true;
					}
					else{
						reachableVertices[a][v][t] = false;
					}
				}
			}
		}
		
	}
	
	
	@Override
	protected void callback() {
		if (where == GRB.CB_MESSAGE) {
			if(optimizationDone == true){
				allThreadFinished = true;
				abort();
			}
//			else{
//				System.out.println("\nGot new message... \n");
//			}
		} 

	}


	public static int[][] solve(Problem p, double timeLimit){
		// Solve the problem
		int paths[][] = null;
		if(p.sg[0].length > 16){
			// 25 Puzzle is best solved using a mixture of methods. 
			PuzzleSolver solver = new PuzzleSolver(p.graph, p.sg[0], p.sg[1]);
			paths = solver.solve(timeLimit);
		}
		else{
			PathPlanner ms = new PathPlanner();
			paths = ms.planPathsAdvanced(p.graph, p.sg[0], p.sg[1], false, 0, true, timeLimit);
		}
		return paths;
	}
	

}
