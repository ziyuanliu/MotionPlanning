package assignment_robots;

import java.util.Arrays;

public class Vertex implements Comparable<Vertex>{
	double[] config;
	private Vertex parent;
	double cost;
	
	public Vertex(double[] c){
		this.config = c;
	}
	
	public void setParent(Vertex p){
		parent = p;
	}
	
	public Vertex getParent(){
		return parent;
	}
	
	public void setCost(double c){
		cost = c;
	}
	
	public static double[] generateRandomConfig(int width, int height){
		return new double[]{Math.random()*width,Math.random()*height,80,Math.random()*2*Math.PI,80,
				Math.random()*2*Math.PI};
	}
	
	public static Vertex generateRandomVertex(int width, int height){
		double[] c = generateRandomConfig(width,height);
		return new Vertex(c);
	}
	
	public int hashCode(){
		return Arrays.hashCode(config);
	}
	
	public String toString(){
		return Arrays.toString(config);
	}
	
	@Override
	public int compareTo(Vertex v){
		return (int) Math.signum(cost - v.cost);
	}
	
	public boolean equals(Vertex o){
		return Arrays.equals(this.config, o.config);
	}
}
