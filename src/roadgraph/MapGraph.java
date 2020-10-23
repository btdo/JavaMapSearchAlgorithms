/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import basicgraph.Graph;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 3
	private Map<GeographicPoint, MapVertex> verticesMap;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		this.verticesMap = new HashMap<GeographicPoint, MapVertex>();
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder();
		for (GeographicPoint point : verticesMap.keySet()) {
			MapVertex node = verticesMap.get(point);
			builder.append(node.toString() + "\n");
		}

		return builder.toString();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		// TODO: Implement this method in WEEK 3
		return verticesMap.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 3
		Set<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		for (GeographicPoint point : verticesMap.keySet()) {
			vertices.add(point);
		}

		return vertices;
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// TODO: Implement this method in WEEK 3
		int numEdges = 0;
		for (GeographicPoint point : verticesMap.keySet()) {
			numEdges += verticesMap.get(point).neighbours.size();
		}

		return numEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location The location of the intersection
	 * @return true if a node was added, false if it was not (the node was already
	 *         in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (verticesMap.containsKey(location) || location == null) {
			return false;
		}
		// TODO: Implement this method in WEEK 3
		MapVertex vertice = new MapVertex(location);
		verticesMap.put(location, vertice);
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from     The starting point of the edge
	 * @param to       The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length   The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been added as
	 *                                  nodes to the graph, if any of the arguments
	 *                                  is null, or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		if (!verticesMap.containsKey(from)) {
			throw new IllegalArgumentException("Points have not already been added");
		}
		// TODO: Implement this method in WEEK 3
		MapVertex source = verticesMap.get(from);
		source.addNeighbour(to, roadName, roadType, length);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization. See assignment instructions for
	 *                     how to use it.
	 * @return The list of intersections that form the shortest (unweighted) path
	 *         from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		Map<MapVertex, MapVertex> vertexFromVertex = new HashMap<MapVertex, MapVertex>();

		MapVertex startVertice = verticesMap.get(start);
		MapVertex goalVertice = verticesMap.get(goal);

		boolean itemFound = searchBFS(startVertice, goalVertice, vertexFromVertex, nodeSearched);
		if (itemFound) {
			return getIntersections(startVertice, goalVertice, vertexFromVertex);
		}

		return null;
	}

	private List<GeographicPoint> getIntersections(MapVertex startVertice, MapVertex goalVertice,
			Map<MapVertex, MapVertex> visited) {
		List<GeographicPoint> items = new ArrayList<GeographicPoint>();
		items.add(goalVertice.getLocation());
		boolean isCompleted = false;
		MapVertex current = goalVertice;
		while (!isCompleted) {
			MapVertex parent = visited.get(current);
			items.add(parent.getLocation());
			current = parent;
			if (parent.equals(startVertice)) {
				isCompleted = true;
			}
		}

		Collections.reverse(items);
		return items;
	}

	private boolean searchBFS(MapVertex startVertex, MapVertex goalVertex, Map<MapVertex, MapVertex> vertexFromAnotherVertex,
			Consumer<GeographicPoint> nodeSearched) {
		Queue<MapVertex> queue = new LinkedList<MapGraph.MapVertex>();
		queue.add(startVertex);
		boolean itemFound = false;
		while (!queue.isEmpty() && !itemFound) {
			MapVertex currentVertex = queue.poll();
			nodeSearched.accept(currentVertex.location);
			if (currentVertex == goalVertex) {
				itemFound = true;
				break;
			}
			
			for (Neighbour neigh : currentVertex.neighbours) {
				if (!vertexFromAnotherVertex.containsKey(neigh.destination)) {
					vertexFromAnotherVertex.put(neigh.destination, currentVertex);
					queue.add(neigh.destination);
				}
			}
		}

		return itemFound;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization. See assignment instructions for
	 *                     how to use it.
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		Map<MapVertex, MapVertex> vertexFromVertex = new HashMap<MapVertex, MapVertex>();
		MapVertex startVertex = verticesMap.get(start);
		MapVertex goalVertex = verticesMap.get(goal);
		boolean itemFound = searchDijkstra(startVertex, goalVertex, vertexFromVertex, nodeSearched);
		if (itemFound) {
			return getIntersections(startVertex, goalVertex, vertexFromVertex);
		}

		return null;
	}
	
	private boolean searchDijkstra(MapVertex startVertex, MapVertex goalVertex, Map<MapVertex, MapVertex> vertextFromAnotherVertex,
			Consumer<GeographicPoint> nodeSearched) {
		Set<MapVertex> visited = new HashSet<MapGraph.MapVertex>();
		Queue<MapVertex> queue = new PriorityQueue<MapGraph.MapVertex>(new MapVertexComparator());
		queue.add(startVertex);
		startVertex.setDistance(0);
		boolean itemFound = false;
		while (!queue.isEmpty() && !itemFound) {
			MapVertex currentVertex = queue.poll();
			if (!visited.contains(currentVertex)) {
				nodeSearched.accept(currentVertex.location);
				visited.add(currentVertex);
				
				if (currentVertex == goalVertex) {
					System.out.println("Visited:" + visited.size());
					return true;
				}
				
				for (Neighbour neigh : currentVertex.neighbours) {
					double currentDistance = currentVertex.distance + neigh.length;
					
					if (!visited.contains(neigh.destination) && currentDistance < neigh.destination.distance ) {
						neigh.destination.distance = currentDistance;
						vertextFromAnotherVertex.put(neigh.destination, currentVertex);
						queue.add(neigh.destination);
					}
					
				}
				
			}
		}
		System.out.println("Visited:" + visited.size());

		return itemFound;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization. See assignment instructions for
	 *                     how to use it.
	 * @return The list of intersections that form the shortest path from start to
	 *         goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		Map<MapVertex, MapVertex> vertexFromVertex = new HashMap<MapVertex, MapVertex>();
		MapVertex startVertex = verticesMap.get(start);
		MapVertex goalVertex = verticesMap.get(goal);
		boolean itemFound = aStarSearch(startVertex, goalVertex, vertexFromVertex, nodeSearched);
		if (itemFound) {
			return getIntersections(startVertex, goalVertex, vertexFromVertex);
		}

		return null;
	}
	
	private boolean aStarSearch(MapVertex startVertex, MapVertex goalVertex, Map<MapVertex, MapVertex> vertextFromAnotherVertex,
			Consumer<GeographicPoint> nodeSearched) {
		Set<MapVertex> visited = new HashSet<MapGraph.MapVertex>();
		Queue<MapVertex> queue = new PriorityQueue<MapGraph.MapVertex>(new MapVertexComparator());
		queue.add(startVertex);
		startVertex.setDistance(0);
		boolean itemFound = false;
		while (!queue.isEmpty() && !itemFound) {
			MapVertex currentVertex = queue.poll();
			if (!visited.contains(currentVertex)) {
				nodeSearched.accept(currentVertex.location);
				visited.add(currentVertex);
				if (currentVertex == goalVertex) {
					System.out.println("Visited:" + visited.size());
					return true;
				}
				
				double minEstimatedDistance = Double.MAX_VALUE;
				
				for (Neighbour neigh : currentVertex.neighbours) {
					double distanceToNeighbour = currentVertex.distance + neigh.length;
					double heuristicDistance = neigh.destination.location.distance(goalVertex.location);
					double estimatedDistanceThroughNeighbour = distanceToNeighbour + heuristicDistance;
					
					if (!visited.contains(neigh.destination) && estimatedDistanceThroughNeighbour < minEstimatedDistance ) {
						minEstimatedDistance = estimatedDistanceThroughNeighbour;
						neigh.destination.distance = distanceToNeighbour;
						vertextFromAnotherVertex.put(neigh.destination, currentVertex);
						queue.add(neigh.destination);
					}
				}
			}
		}
		
		System.out.println("Visited:" + visited.size());
		return itemFound;
	}
	
	private static void printIntersection(List<GeographicPoint> testroute) {
		if (testroute != null) {
			for (GeographicPoint intersection: testroute) {
				System.out.println(intersection.toString() + "\n");
			}
		}
	}

	public static void main(String[] args) {
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
//		System.out.println(firstMap.toString());
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		List<GeographicPoint> testroute = firstMap.bfs(testStart, testEnd);
//		System.out.println("Bfs.");
//		if (testroute != null) {
//			for (GeographicPoint intersection: testroute) {
//				System.out.println(intersection.toString() + "\n");
//			}
//		}
//		
//		List<GeographicPoint> testroute2 = firstMap.dijkstra(testStart, testEnd);
//		System.out.println("Dijkstra.");
//		if (testroute2 != null) {
//			for (GeographicPoint intersection: testroute2) {
//				System.out.println(intersection.toString() + "\n");
//			}
//		}
		

		// You can use this method for testing.

		/*
		 * Here are some test cases you should try before you attempt the Week 3 End of
		 * Week Quiz, EVEN IF you score 100% on the programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		System.out.println("Bfs.");
		List<GeographicPoint> testroute0 = simpleTestMap.bfs(testStart, testEnd);
		printIntersection(testroute0);
		System.out.println("Dijkstra.");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
		printIntersection(testroute);
		System.out.println("A*");
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);
		printIntersection(testroute2);
		
		

		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

		// A very simple test using real data 
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		System.out.println("Dijkstra.");
		testroute = testMap.dijkstra(testStart, testEnd);
		printIntersection(testroute);
		System.out.println("A*");
		testroute2 = testMap.aStarSearch(testStart, testEnd);
		printIntersection(testroute2);

		// A slightly more complex test using real data 
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		System.out.println("Dijkstra.");
		testroute = testMap.dijkstra(testStart, testEnd);
		printIntersection(testroute);
		System.out.println("A*");
		testroute2 = testMap.aStarSearch(testStart, testEnd);
		printIntersection(testroute2);

		/* Use this code in Week 3 End of Week Quiz */
		/*
		 * MapGraph theMap = new MapGraph();
		 * System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		 * System.out.println("DONE.");
		 * 
		 * GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		 * GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		 * 
		 * 
		 * List<GeographicPoint> route = theMap.dijkstra(start,end);
		 * List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 * 
		 */

	}

	private class MapVertex {
		private double distance = Double.MAX_VALUE;
		private GeographicPoint location;
		private List<Neighbour> neighbours;

		public MapVertex(GeographicPoint location) {
			this.location = location;
			this.neighbours = new LinkedList<MapGraph.Neighbour>();
		}

		public void addNeighbour(GeographicPoint to, String roadName, String roadType, double length) {
			if (!verticesMap.containsKey(to)) {
				throw new IllegalArgumentException("Points have not already been added");
			}

			MapVertex destination = verticesMap.get(to);
			Neighbour neighbour = new Neighbour(destination, roadName, roadType, length);
			this.neighbours.add(neighbour);
		}

		public GeographicPoint getLocation() {
			return location;
		}

		public List<Neighbour> getNeighbours() {
			return neighbours;
		}

		@Override
		public String toString() {
			StringBuffer str = new StringBuffer();

			str.append("source=" + location.toString() + "\n").append("neighbours: \n").toString();

			for (Neighbour neighbour : neighbours) {
				str.append("  " + neighbour.toString() + "\n");
			}

			return str.toString();
		}
		

		public double getDistance() {
			return distance;
		}

		public void setDistance(double distance) {
			this.distance = distance;
		}

		@Override
		public boolean equals(Object obj) {
			if (obj instanceof MapVertex) {
				return this.location.equals(((MapVertex) obj).location);
			}
			
			return false;
		}
		
		@Override
		public int hashCode()
		{
			return location.hashCode();
		}
	}

	private class Neighbour {
		MapVertex destination;
		String roadName;
		String roadType;
		double length;

		public Neighbour(MapVertex destination, String roadName, String roadType, double length) {
			this.destination = destination;
			this.roadName = roadName;
			this.roadType = roadType;
			this.length = length;
		}

		@Override
		public String toString() {
			return new StringBuffer().append("destination=" + destination.getLocation().toString())
					.append(" roadName=" + roadName).append(" roadType=" + roadType).append(" length=" + length)
					.toString();
		}
	}
	
	private class MapVertexComparator implements Comparator<MapVertex> {

		@Override
		public int compare(MapVertex o1, MapVertex o2) {
			// TODO Auto-generated method stub
			return o1.distance > o2.distance ? 1 : -1;
		}

		
		
	}

}
