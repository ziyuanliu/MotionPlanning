package assignment_robots;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.shape.Polygon;
import javafx.scene.Group;
import javafx.stage.Stage;
import javafx.scene.paint.Color;

public class ArmDriver extends Application {
	// default window size
	protected int window_width = 600;
	protected int window_height = 400;
	
	public void addPolygon(Group g, Double[] points) {
		Polygon p = new Polygon();
	    p.getPoints().addAll(points);
	    
	    g.getChildren().add(p);
	}
	
	// plot a ArmRobot;
	public void plotArmRobot(Group g, ArmRobot arm, double[] config, Color c) {
		arm.set(config);
		double[][] current;
		Double[] to_add;
		Polygon p;
		for (int i = 1; i <= arm.getLinks(); i++) {
			current = arm.getLinkBox(i);
			
			
			to_add = new Double[2*current.length];
			for (int j = 0; j < current.length; j++) {
//				System.out.println(current[j][0] + ", " + current[j][1]);
				to_add[2*j] = current[j][0];
				//to_add[2*j+1] = current[j][1];
				to_add[2*j+1] = window_height - current[j][1];
			}
			p = new Polygon();
			p.getPoints().addAll(to_add);
			p.setStroke(Color.BLUE);
			p.setFill(c);
			g.getChildren().add(p);
		}
		
	}
	
	
	public void plotWorld(Group g, World w) {
		int len = w.getNumOfObstacles();
		double[][] current;
		Double[] to_add;
		Polygon p;
		for (int i = 0; i < len; i++) {
			current = w.getObstacle(i);
			to_add = new Double[2*current.length];
			for (int j = 0; j < current.length; j++) {
				to_add[2*j] = current[j][0];
				//to_add[2*j+1] = current[j][1];
				to_add[2*j+1] = window_height - current[j][1];
			}
			p = new Polygon();
			p.getPoints().addAll(to_add);
			g.getChildren().add(p);
		}
	}
	
	// The start function; will call the drawing;
	// You can run your PRM or RRT to find the path; 
	// call them in start; then plot the entire path using
	// interfaces provided;
	@Override
	public void start(Stage primaryStage) {
		
		
		// setting up javafx graphics environments;
		primaryStage.setTitle("CS 76 2D world");

		Group root = new Group();
		Scene scene = new Scene(root, window_width, window_height);

		primaryStage.setScene(scene);
		
		Group g = new Group();

		// setting up the world;
		
		// creating polygon as obstacles;
		

		double a[][] = {{10, 400}, {150, 300}, {100, 210}};
		Poly obstacle1 = new Poly(a);
		
		double b[][] = {{400, 10}, {500, 120}, {520, 45}};

		Poly obstacle2 = new Poly(b);
		
		
		double c[][] = {{210, 50}, {350, 280}, {360, 50}};
		Poly obstacle3 = new Poly(c);

		double bottom[][] = {{0, 0}, {this.window_width, 0}, {this.window_width,-100},{0,-100}};
		Poly bottomWall = new Poly(bottom);
		
		double right[][] = {{this.window_width, this.window_height}, {this.window_width+100, this.window_height}, 
				{this.window_width+100, 0},{this.window_width,0}};
		Poly rightWall = new Poly(right);
	
		
		// Declaring a world; 
		World w = new World();
		// Add obstacles to the world;
		w.addObstacle(obstacle1);
		w.addObstacle(obstacle2);
		w.addObstacle(obstacle3);
		w.addObstacle(bottomWall);
		w.addObstacle(rightWall);
		
		plotWorld(g, w);
		
		ArmRobot arm = new ArmRobot(2);
		
		double[] config1 = {10, 20, 80, Math.PI/4, 80, Math.PI/4};
		double[] config2 = {400, 250, 80, .1, 80, .2};
		
		arm.set(config2);
		
		// Plan path between two configurations;
		ArmLocalPlanner ap = new ArmLocalPlanner();
		
		//Random sampling
		int n = 100; //<< number of random samples
		int k = 15; //Number for nearest - k
		
		ArrayList<Vertex> vertices = new ArrayList<Vertex>();
		
		//add the start and goal
		Vertex start = new Vertex(config1);
		Vertex goal = new Vertex(config2);
		goal.setCost(Double.MAX_VALUE);
		
		while(n!=0){
			Vertex v = Vertex.generateRandomVertex(this.window_width, this.window_height);
			arm.set(v.config);
			
			if(!w.armCollision(arm)){
				n--;
				v.setCost(Double.MAX_VALUE);
				vertices.add(v);
			}
		}
		
		//get the path
		List<Vertex> path = dijkstraSearch(vertices, start, goal, k, ap, w, arm);
		if(path!=null){
			drawPath(path,g,arm,ap);
		}
		
		
	    scene.setRoot(g);
	    primaryStage.show();
		

	}
	
	public void drawPath(List<Vertex> path, Group g, ArmRobot arm, ArmLocalPlanner ap){
		for(int i = 0; i < path.size(); i ++){
			arm.set(path.get(i).config);
			plotArmRobot(g, arm, path.get(i).config,Color.BEIGE);
			if(i+1<path.size()){
				double[] base = path.get(i).config;
				double[] rate = ap.getPath(path.get(i).config, path.get(i+1).config);
				double time = ap.moveInParallel(path.get(i).config, path.get(i+1).config);
				for(double j = 0; j < time; j+=10){
					for(int x = 0; x < base.length; x++){
						base[x]+=10*rate[x];
					}
					arm.set(base);
					plotArmRobot(g, arm, base,Color.ALICEBLUE);
				}
			}
		}
	}
	
	public List<Vertex> dijkstraSearch(ArrayList<Vertex> vertices, Vertex start, 
			Vertex goal, int k, ArmLocalPlanner ap, World w, ArmRobot arm){
		PriorityQueue<Vertex> frontier = new PriorityQueue<Vertex>();

		start.setCost(0);
		goal.setCost(Double.MAX_VALUE);
		frontier.add(start);
		vertices.add(goal);
		HashMap<Vertex,Double> visitedCosts = new HashMap<Vertex,Double>();
		while(!frontier.isEmpty()){
			Vertex currentV = frontier.remove();
			arm.set(currentV.config);

			if(goal.equals(currentV)){
				return backChain(currentV);
			}else if(currentV.cost==Double.MAX_VALUE){
				break;
			}
			
			for(Vertex n: knn(k,currentV,vertices,ap,w,arm)){
				if(!visitedCosts.containsKey(n)||n.cost<visitedCosts.get(n)){
					double dist = currentV.cost+ap.moveInParallel(currentV.config, n.config);
	
					if (dist<n.cost){
						n.setCost(dist);
						n.setParent(currentV);
						frontier.add(n);
					}
				}
			}
			visitedCosts.put(currentV,currentV.cost);
		}
		
		return null;
	}
	
	public List<Vertex> backChain(Vertex v){
		LinkedList<Vertex> solution = new LinkedList<Vertex>();

		while (v.getParent() != null) {
			solution.addFirst(v);
			v = v.getParent();
		}
		solution.addFirst(v);
		return solution;
	}
	
	public ArrayList<Vertex> knn(int k, Vertex vertex, List<Vertex> vertices, ArmLocalPlanner ap, World w, ArmRobot arm){
		PriorityQueue<Vertex> kF = new PriorityQueue<Vertex>(k, new NeighborComparator(vertex));
		kF.addAll(vertices);
		
		int ctr = 0;
		ArrayList<Vertex> retval = new ArrayList<Vertex>();
		
		while(ctr<k&&!kF.isEmpty()){
			Vertex v = kF.remove();
			arm.set(v.config);
			
			if(!w.armCollisionPath(arm, vertex.config, v.config)){
				retval.add(v);
				ctr++;
			}
		}
		
		return retval;
	}
	
	public class NeighborComparator implements Comparator<Vertex>{
		Vertex current;
		ArmLocalPlanner ap;
		ArmRobot arm;
		public NeighborComparator(Vertex v){
			 ap = new ArmLocalPlanner();
			 current = v;
			 arm = new ArmRobot(2);
		}
		
		@Override
		public int compare(Vertex o1, Vertex o2) {
			
			if(closeness(o1.config, current.config)>closeness(o2.config, current.config)){
				return 1;
			}
			if(closeness(o1.config, current.config)<closeness(o2.config, current.config)){
				return -1;
			}
			return 0;
		}
		
		public double closeness(double[] o, double[] o1){
			double weight = Math.abs(o[3]-o1[3])+ Math.abs(o[5]-o1[5]);
			return Math.sqrt(Math.pow(o[0]-o1[0], 2)+Math.pow(o[1]-o1[1], 2))+weight;
		}
	}
	
	public static void main(String[] args) {
		launch(args);
	}
}
