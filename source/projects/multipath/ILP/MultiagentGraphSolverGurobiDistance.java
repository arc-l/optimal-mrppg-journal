package projects.multipath.ILP;

import gurobi.GRB;
import gurobi.GRBCallback;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;

import java.util.HashSet;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;

import projects.multipath.advanced.Graph;

public class MultiagentGraphSolverGurobiDistance extends GRBCallback{

	public static boolean bPrintPaths = false;
	public static boolean bDebugInfo = true;
	public static boolean bReturnAnySolution = false;
	
	private SortedMap<Long, GRBVar> edgeVarMap = new TreeMap<Long, GRBVar>();
	private SortedMap<Long, Vector<GRBVar>> vertexTimeInVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
	private SortedMap<Long, Vector<GRBVar>> vertexTimeOutVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
	private SortedMap<Long, Vector<GRBVar>> avtInVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
	private SortedMap<Long, Vector<GRBVar>> avtOutVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
	private SortedMap<Long, Vector<GRBVar>> edgeTimeVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
	private boolean reachableVertices[][][] = null;
	private GRBModel currentModel = null;
	private int expectedBound = 0;
			
	protected int[][] solve(Graph graph, int starts[], int goals[], double gap, int expansion, double timeLimit) throws GRBException{

		long startTime = System.currentTimeMillis(); 
		updateReachableSets(graph, starts, goals, expansion);
		
		// Create LP solver
		GRBEnv env = new GRBEnv();
		env.set(GRB.IntParam.OutputFlag, 0);
		env.set(GRB.DoubleParam.MIPGap, gap);

		// Solve the ILP
		int ps[][] = null;
		// timeLimit = timeLimit - (System.currentTimeMillis() - startTime)/1000.0;
		if(bDebugInfo){
			System.out.print("Trying T = " + expansion + " for the integer model " + ((timeLimit != -1)?("with max time limit " + timeLimit):"") + ", gap=" + gap + " ...");
		}
		GRBModel model = prepareModel(env, graph, starts, goals, true, expansion, timeLimit);
		expectedBound = starts.length;
		model.optimize();
		double objVal = 0;
		if(model.get(GRB.IntAttr.SolCount) > 0){
			objVal = (model.get(GRB.DoubleAttr.ObjVal));
			double bnd = model.get(GRB.DoubleAttr.ObjBound);
			double agap = 1 - bnd/objVal;
			
			if(agap <= gap || bReturnAnySolution){
				if(bDebugInfo){
					System.out.println(" done.\n");
					System.out.println("Solution with optimal distance " + objVal + " found in " + (System.currentTimeMillis() - startTime)/1000.0 + " seconds.");
				}
				ps = retrievePaths(model, graph, starts, expansion);
			}
		}
		if(bDebugInfo && ps == null){
			System.out.println(" time out.");
		}
		
		model.dispose();
		env.dispose();
		
		return ps;
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
	
	private long getEdgeId(int a, int v1, int v2, int t) {
		return a * 1000000000000L + v1 * 100000000 + v2 * 10000 + t;
	}

	private long getVertexTimeId(int v, int t) {
		return v * 10000 + t;
	}

	private long getAgentVertexTimeId(int a, int v, int t) {
		return a * 1000000000000L + v * 10000 + t;
	}

	private long getEdgeTimeId(int v1, int v2, int t) {
		if(v1 > v2){
			int temp = v1; 
			v1 = v2;
			v2 = temp;
		}
		return v1 * 1000000000 + v2* 10000 + t;
	}

	private int[][] retrievePaths(GRBModel model, Graph graph, int starts[], int timeSteps) throws GRBException{
		int paths[][] = new int[starts.length][timeSteps + 1];
		for(int a = 0; a < starts.length; a ++){
			if(bPrintPaths){
				System.out.print("Agent " + a + ": 0:");
				graph.vertices[starts[a]].printVertex();
			}
			paths[a][0] = starts[a];
			for(int t = 1; t <= timeSteps; t ++){
				Integer nvs[] = graph.adjacencySetMap.get(paths[a][t-1]).toArray(new Integer[0]);
				for(int nv = 0; nv < nvs.length; nv ++){
					if(isEdgeVarSetToTrue(model, a, paths[a][t-1], nvs[nv], t-1)){
						paths[a][t] = nvs[nv];
						break;
					}
				}
				if(bPrintPaths){
					System.out.print(" " + t + ":");
					graph.vertices[paths[a][t]].printVertex();
				}
			}
			if(bPrintPaths)System.out.println();
		}
		return paths;
	}
	
	private boolean isEdgeVarSetToTrue(GRBModel model, int a, int v1, int v2, int t) throws GRBException{
		long eId = getEdgeId(a, v1, v2, t);
		GRBVar var = edgeVarMap.get(eId);
		if(var != null && var.get(GRB.DoubleAttr.X) == 1.0){
			return true;
		}
		return false;
	}
	
	private GRBModel prepareModel(GRBEnv env, Graph graph, int starts[], int goals[], boolean integer, int timeSteps, double timeLimit) throws GRBException{
		// Reset global storage
		edgeVarMap = new TreeMap<Long, GRBVar>();
		vertexTimeInVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
		vertexTimeOutVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
		avtInVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
		avtOutVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
		edgeTimeVarVectorMap = new TreeMap<Long, Vector<GRBVar>>();
		
		// Suppress output if we are solving the relaxation
		if(bDebugInfo){env.set(GRB.IntParam.OutputFlag, 0);}
		else{env.set(GRB.IntParam.OutputFlag, 0);}

		// Time limit?
		if(timeLimit > 0)
		{ 
			env.set(GRB.DoubleParam.TimeLimit, timeLimit); 
		}
		else{
			env.set(GRB.DoubleParam.TimeLimit, 10000000000.0);
		}
		
		// Setup model
		GRBModel model = new GRBModel(env);
		currentModel = model;

		GRBLinExpr objExpr = new GRBLinExpr();

		// Set up variables for all edges, agent by agent
		for(int a = 0; a < starts.length; a++){
			Set<Integer> rVs = new HashSet<Integer>();
			rVs.add(starts[a]);
			for(int t = 1; t <= timeSteps; t++){
				Integer[] rvs = rVs.toArray(new Integer[0]);
				for(int rv = 0; rv < rvs.length; rv ++){
					int curV = rvs[rv];
					Integer[] adjVs = graph.adjacencySetMap.get(curV).toArray(new Integer[0]);
					for(int i = 0; i < adjVs.length; i ++){
						int nextV = adjVs[i];
						if(reachableVertices[a][nextV][t]){
							rVs.add(nextV);
							GRBVar x = model.addVar(0.0, 1.0, 0.0, integer?GRB.BINARY:GRB.CONTINUOUS, null);
							if(curV != nextV){
								objExpr.addTerm(1.0, x);
							}
							long edgeId = getEdgeId(a, curV, nextV, t - 1);
							edgeVarMap.put(edgeId, x);

							// Store edges indexed by vertices
							storeVarForVertices(x, curV, nextV, t - 1, t);
							storeVarForAVT(x, a, curV, nextV, t - 1, t);

							// Store edges indexed by edges, if the start/goal vertice are different
							if(curV != nextV){
								storeVarForEdges(x, curV, nextV, t - 1);
							}
						}
					}

					// Remove current vertex if it's no longer reachable
					if(!reachableVertices[a][curV][t]){
						rVs.remove(curV);
					}
				}
			}
		}
		model.update();
		model.setObjective(objExpr, GRB.MINIMIZE);
		
		// Special case for last t, only need to setup single edges between respective start/goal
		for(int a = 0; a < starts.length; a ++){
			long edgeId = getEdgeId(a, goals[a], starts[a], timeSteps);
			GRBVar x = model.addVar(integer?1.0:0.0, 1.0, 0.0, integer?GRB.BINARY:GRB.CONTINUOUS, null);
			model.update();

			// Store edges indexed by vertices
			storeVarForVertices(x, goals[a], starts[a], timeSteps, 0);
			storeVarForAVT(x, a, goals[a], starts[a], timeSteps, 0);
			
			// Setup the objective exprssion 
			edgeVarMap.put(edgeId, x);
		}
		

		// Set up constraints for edges going into a vertex
		for(int t = 0; t < timeSteps; t++){

			// Set up variables for all edges and expression 
			for(int v = 0; v < graph.vertices.length; v++){
				GRBLinExpr expr = null;
				long vtId = getVertexTimeId(v, t + 1);
				Vector<GRBVar> inVarVec = vertexTimeInVarVectorMap.get(vtId);
				if(inVarVec != null){
					if(expr == null) expr = new GRBLinExpr();
					for(int i = 0; i < inVarVec.size(); i ++){
						expr.addTerm(1., inVarVec.get(i));
					}
				}
				if(expr != null){
					if(expr.size() > 1){
						model.addConstr(expr, GRB.LESS_EQUAL, 1, null);
					}
					expr = null;
				}
			}
		}
		
		// Set up constraints for vertices
		for(int a = 0; a < starts.length; a ++){
			for(int t = 0; t <= timeSteps; t++){
				// Set up variables for all edges and expression 
				for(int v = 0; v < graph.vertices.length; v++){
					GRBLinExpr expr = null;
					long vtId = getAgentVertexTimeId(a, v, t);
					Vector<GRBVar> inVarVec = avtInVarVectorMap.get(vtId);
					if(inVarVec != null){
						expr = new GRBLinExpr();
						for(int i = 0; i < inVarVec.size(); i ++){
							expr.addTerm(-1., inVarVec.get(i));
						}
					}
					Vector<GRBVar> outVarVec = avtOutVarVectorMap.get(vtId);
					if(outVarVec != null){
						if(expr == null) expr = new GRBLinExpr();
						for(int i = 0; i < outVarVec.size(); i ++){
							expr.addTerm(1., outVarVec.get(i));
						}
					}
					if(expr != null){
						model.addConstr(expr, GRB.EQUAL, 0, null);
						expr = null;
					}
				}
			}
		}
		
		// Setup constraint for single edges
		Long[] edgeTimeKeys = edgeTimeVarVectorMap.keySet().toArray(new Long[0]);
		for(int e = 0; e < edgeTimeKeys.length; e ++){
			GRBLinExpr expr = null;
			Vector<GRBVar> varVec = edgeTimeVarVectorMap.get(edgeTimeKeys[e]);
			if(varVec != null){
				if(expr == null) expr = new GRBLinExpr();
				for(int i = 0; i < varVec.size(); i ++){
					expr.addTerm(1., varVec.get(i));
				}
			}
			if(expr != null && expr.size() > 1){
				model.addConstr(expr, GRB.LESS_EQUAL, 1, null);
			}
			expr = null;
		}
		model.update();
		
		return model;
	}
	
	private void storeVarForAVT(GRBVar var, int a, int v1, int v2, int t1, int t2){
		// Add edge var to vertex time map, outgoing first
		long vtId = getAgentVertexTimeId(a, v1, t1);
		Vector<GRBVar> varVec = avtOutVarVectorMap.get(vtId);
		if(varVec == null){
			varVec = new Vector<GRBVar>();
			avtOutVarVectorMap.put(vtId, varVec);
		}
		varVec.add(var);
		
		// Incoming
		vtId = getAgentVertexTimeId(a, v2, t2);
		varVec = avtInVarVectorMap.get(vtId);
		if(varVec == null){
			varVec = new Vector<GRBVar>();
			avtInVarVectorMap.put(vtId, varVec);
		}
		varVec.add(var);
	}

	/**
	 * Store edge var for head and tail vertices
	 * @param var
	 * @param v1
	 * @param v2
	 * @param t1
	 * @param t2
	 */
	private void storeVarForVertices(GRBVar var, int v1, int v2, int t1, int t2){
		// Add edge var to vertex time map, outgoing first
		long vtId = getVertexTimeId(v1, t1);
		Vector<GRBVar> varVec = vertexTimeOutVarVectorMap.get(vtId);
		if(varVec == null){
			varVec = new Vector<GRBVar>();
			vertexTimeOutVarVectorMap.put(vtId, varVec);
		}
		varVec.add(var);
		
		// Incoming
		vtId = getVertexTimeId(v2, t2);
		varVec = vertexTimeInVarVectorMap.get(vtId);
		if(varVec == null){
			varVec = new Vector<GRBVar>();
			vertexTimeInVarVectorMap.put(vtId, varVec);
		}
		varVec.add(var);
	}

	/**
	 * Store edge var for one edge in the original graph with different time stamps
	 * @param var
	 * @param v1
	 * @param v2
	 * @param t
	 */
	private void storeVarForEdges(GRBVar var, int v1, int v2, int t){
		// Add edge var to edge time map
		long etId = getEdgeTimeId(v1, v2, t);
		Vector<GRBVar> varVec = edgeTimeVarVectorMap.get(etId);
		if(varVec == null){
			varVec = new Vector<GRBVar>();
			edgeTimeVarVectorMap.put(etId, varVec);
		}
		varVec.add(var);
	}
	
	@Override
	protected void callback() {
		try{
			if (where == GRB.CB_MIP) {
				if(expectedBound > getDoubleInfo(GRB.Callback.MIP_OBJBND)){
					currentModel.terminate();
				}
			}
		} catch (GRBException e) {
			System.out.println("Error code: " + e.getErrorCode() + ". "
					+ e.getMessage());
			e.printStackTrace();
		}

	}


}
