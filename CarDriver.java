package assignment_robots;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.shape.Line;
import javafx.scene.shape.Polygon;
import javafx.scene.Group;
import javafx.stage.Stage;
import javafx.scene.paint.Color;

public class CarDriver extends Application {
	// default window size
	protected int window_width = 600;
	protected int window_height = 400;
	
	// Draw a polygon;
	public void addPolygon(Group g, Double[] points) {
		Polygon p = new Polygon();
	    p.getPoints().addAll(points);
	    
	    g.getChildren().add(p);
	}
	
	// plot a car robot
	public void plotCarRobot(Group g, CarRobot car, CarState s) {
		//System.out.println(car);
		//System.out.println(s);
		car.set(s); 
		double[][] current = car.get();
		Double[] to_add = new Double[2*current.length];
		for (int j = 0; j < current.length; j++) {
//			System.out.println(current[j][0] + ", " + current[j][1]);
			to_add[2*j] = current[j][0];
			//to_add[2*j+1] = current[j][1];
			to_add[2*j+1] = window_height - current[j][1];
		}
		Polygon p = new Polygon();
		p.getPoints().addAll(to_add);
		
		p.setStroke(Color.RED);
		p.setFill(Color.PINK);
		g.getChildren().add(p);
	}
		
	public void plotEdge(Group g, CarState s, CarState s1){
		Line l = new Line();
		l.setStroke(Color.AQUA);
		l.setStrokeWidth(0.5);
		l.setStartX(s.getX());
		l.setStartY(window_height-s.getY());
		l.setEndX(s1.getX());
		l.setEndY(window_height-s1.getY());
		g.getChildren().add(l);
	}
	// plot the World with all the obstacles;
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
		
		
	
		primaryStage.setTitle("CS 76 2D world");
		Group root = new Group();
		Scene scene = new Scene(root, window_width, window_height);

		primaryStage.setScene(scene);
		
		Group g = new Group();
		
		double a[][] = {{0, 300}, {400, 300}, {400, 275},{0,275}};
		Poly obstacle1 = new Poly(a);
		
		double b[][] = {{50, 225}, {600, 225}, {600, 200},{50,200}};

		Poly obstacle2 = new Poly(b);

		
		double c[][] = {{0, 150}, {400, 150}, {400, 125},{0,125}};
		Poly obstacle3 = new Poly(c);
		
		double d[][] = {{50, 75}, {600, 75}, {600, 50},{50,50}};
		Poly obstacle4 = new Poly(d);
		
		double e[][] = {{0,0}, {0,this.window_height},{10,this.window_height},{10,0}};
		Poly obstacle5 = new Poly(e);
		
		double f[][] = {{this.window_width-10,0}, {this.window_width-10,this.window_height},
				{this.window_width,this.window_height},{this.window_width,0}};
		Poly obstacle6 = new Poly(f);
		
		// Declaring a world; 
		World w = new World();
		// Add obstacles to the world;
		w.addObstacle(obstacle1);
		w.addObstacle(obstacle2);
		w.addObstacle(obstacle3);
		w.addObstacle(obstacle4);
		w.addObstacle(obstacle5);
		w.addObstacle(obstacle6);
			
		plotWorld(g, w);
		
		CarRobot car = new CarRobot();
		CarRobot car1 = new CarRobot();
		CarState state1 = new CarState(270, 15, 0);
		CarState state2 = new CarState(440, 300, Math.PI/2);
	    // Set CarState;
		car.set(state1);
		car1.set(state2);
	    
		plotCarRobot(g, car, state1);
		plotCarRobot(g, car1, state2);
		
		ArrayList<CarState> path = (ArrayList<CarState>) RRT(car,car1,w, g, scene, primaryStage);
		if(path!=null){
			System.out.println(path);
//			drawPath(path,g,car);
		}
	    scene.setRoot(g);
	   
	    primaryStage.show();
		
	}
	public static void main(String[] args) {
		launch(args);
	}
	
	public List<CarState> RRT(CarRobot initial, CarRobot goal, World w, Group g, 
			Scene scene, Stage primaryStage){
		HashMap<CarState,CarState> visitedFrom = new HashMap<CarState,CarState>();
		CarState min=null;
		visitedFrom.put(initial.getCarState(), null);
		
		do{
			CarRobot rand = CarRobot.randomCar(this.window_width, this.window_height);
			CarRobot q = nearest(rand.getCarState(),visitedFrom);
			SteeredCar car = new SteeredCar();

			min = null;
			CarRobot tempQ = new CarRobot();

			while(min==null){
				double dst = Double.MAX_VALUE;
				double duration = 5.0*Math.random();
				
				for(int i = 0; i < 6; i++){
					tempQ.set(q.getCarState());
					
					CarState nextState = car.move(tempQ.getCarState(), i, duration);
					double tempDist = dst(nextState,rand.getCarState());
					
					if(tempDist<dst&&!w.carCollisionPath(tempQ, tempQ.getCarState(), i, duration)){
						min = nextState;
						dst = tempDist;
						plotEdge(g,q.getCarState(),nextState);
					}
					
				}
			}
			scene.setRoot(g);
			System.out.println("goign "+min);
		    primaryStage.show();
		    if(!visitedFrom.containsKey(min))
		    	visitedFrom.put(min, q.getCarState());
		}while(!inGoalVicinity(goal.getCarState(),min));
		
		visitedFrom.put(goal.getCarState(), min);
		return backChain(goal.getCarState(),visitedFrom);
	}
	
	public boolean inGoalVicinity(CarState goal, CarState current){
		return (5>dst(goal, current));
	}
	
	public void drawPath(List<CarState> path, Group g, CarRobot car){
		for(CarState cs:path){
			car.set(cs);
			plotCarRobot(g,car,cs);
		}
	}
	
	public List<CarState> backChain(CarState car, HashMap<CarState,CarState> graph){
		for(CarState c : graph.keySet()){
			System.out.println(c+" "+c.hashCode()+" v="+graph.get(c));
		}
		ArrayList<CarState> solution = new ArrayList<CarState>();

		// chain through the visited hashmap to find each previous node,
		// add to the solution
		while (car != null) {
			System.out.println(car+" "+car.hashCode()+" "+graph.containsKey(car));
			solution.add(0,car);
			car = graph.get(car);
		}

		return solution;
	}
	public CarRobot nearest(CarState rand, HashMap<CarState,CarState> list){
		double dst = Double.MAX_VALUE;
		CarState min=null;
		for(CarState car:list.keySet()){
			if(dst(car,rand)<dst){
				min = car;
				dst = dst(car,rand);
			}
		}
		CarRobot retval = new CarRobot();
		retval.set(min);
		return retval;
	}
	
	public double dst(CarState o, CarState o1){
		double angle = Math.abs(o.getTheta()-o.getTheta());
		return Math.sqrt(angle)+Math.sqrt(Math.pow(o.getX()-o1.getX(), 2)+Math.pow(o.getY()-o1.getY(), 2));
	}
}
