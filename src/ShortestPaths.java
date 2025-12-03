import java.util.Map;
import java.util.PriorityQueue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.io.File;
import java.io.FileNotFoundException;

/** Provides an implementation of Dijkstra's single-source shortest paths
 * algorithm.
 * Sample usage:
 *   Graph g = // create your graph
 *   ShortestPaths sp = new ShortestPaths();
 *   Node a = g.getNode("A");
 *   sp.compute(a);
 *   Node b = g.getNode("B");
 *   LinkedList<Node> abPath = sp.getShortestPath(b);
 *   double abPathLength = sp.getShortestPathLength(b);
 *   */
public class ShortestPaths {
    // stores auxiliary data associated with each node for the shortest
    // paths computation:
    private HashMap<Node,PathData> paths;

    /** Compute the shortest path to all nodes from origin using Dijkstra's
     * algorithm. Fill in the paths field, which associates each Node with its
     * PathData record, storing total distance from the source, and the
     * back pointer to the previous node on the shortest path.
     * Precondition: origin is a node in the Graph.*/
    public void compute(Node origin) {
        this.paths = new HashMap<>();
        PriorityQueue<Node> pQueue = new PriorityQueue<>(
            (Node a, Node b) -> (int) (paths.get(a).distance - paths.get(b).distance));

        paths.put(origin, new PathData((double) 0, null));
        pQueue.add(origin);
        HashSet<Node> visited = new HashSet<>();

        do {
            Node current = pQueue.poll();
            if (visited.contains(current)) {
                continue;
            }

            HashMap<Node, Double> neighbors = current.getNeighbors();

            for (Node x : neighbors.keySet()) {
                if (paths.containsKey(x)) {
                    double temp = paths.get(current).distance + neighbors.get(x);

                    if (temp < paths.get(x).distance) {
                        paths.put(x, new PathData(temp, current));
                    }
                    continue;
                }
                else {
                    paths.put(x, new PathData(neighbors.get(x), current));
                    pQueue.add(x);
                }
            }
            visited.add(current);

        } while (pQueue.size() > 0);
    }

    /** Returns the length of the shortest path from the origin to destination.
     * If no path exists, return Double.POSITIVE_INFINITY.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public double shortestPathLength(Node destination) {
        if (paths.containsKey(destination)) {
            return paths.get(destination).distance;
        }
        return Double.POSITIVE_INFINITY;
    }

    /** Returns a LinkedList of the nodes along the shortest path from origin
     * to destination. This path includes the origin and destination. If origin
     * and destination are the same node, it is included only once.
     * If no path to it exists, return null.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public LinkedList<Node> shortestPath(Node destination) {
        if (!paths.containsKey(destination)) {
            return null;
        }
        
        Node current = destination;
        LinkedList<Node> path = new LinkedList<>();

        while (current != null) {
            path.addFirst(current);
            current = paths.get(current).previous;
        }
        return path;
    }


    /** Inner class representing data used by Dijkstra's algorithm in the
     * process of computing shortest paths from a given source node. */
    class PathData {
        double distance; // distance of the shortest path from source
        Node previous; // previous node in the path from the source

        /** constructor: initialize distance and previous node */
        public PathData(double dist, Node prev) {
            distance = dist;
            previous = prev;
        }
    }


    /** Static helper method to open and parse a file containing graph
     * information. Can parse either a basic file or a CSV file with
     * sidewalk data. See GraphParser, BasicParser, and DBParser for more.*/
    protected static Graph parseGraph(String fileType, String fileName) throws
        FileNotFoundException {
        // create an appropriate parser for the given file type
        GraphParser parser;
        if (fileType.equals("basic")) {
            parser = new BasicParser();
        } else if (fileType.equals("db")) {
            parser = new DBParser();
        } else {
            throw new IllegalArgumentException(
                    "Unsupported file type: " + fileType);
        }

        // open the given file
        parser.open(new File(fileName));

        // parse the file and return the graph
        return parser.parse();
    }

    public static void main(String[] args) {
      // read command line args
      String fileType = args[0];
      String fileName = args[1];
      String SidewalkOrigCode = args[2];

      String SidewalkDestCode = null;
      if (args.length == 4) {
        SidewalkDestCode = args[3];
      }

      // parse a graph with the given type and filename
      Graph graph;
      try {
          graph = parseGraph(fileType, fileName);
      } catch (FileNotFoundException e) {
          System.out.println("Could not open file " + fileName);
          return;
      }
      graph.report();


      ShortestPaths sPaths = new ShortestPaths();
      Node origin = graph.getNode(SidewalkOrigCode);
      sPaths.compute(origin);
      if (SidewalkDestCode == null) {
      // TODO 5:
      // If destCode was not given, print each reachable node followed by the
      // length of the shortest path to it from the origin.

      } else {
        Node dest = graph.getNode(SidewalkDestCode);
        System.out.println();
        for (Node node : sPaths.shortestPath(dest)) {
            System.out.printf("Node %s -> ", node.getId());
        }
        System.out.printf("\nPath length: %s\n", sPaths.shortestPathLength(dest));
      }

      // TODO 6:
      // If destCode was given, print the nodes in the path from
      // origCode to destCode, followed by the total path length
      // If no path exists, print a message saying so.
    }
}
