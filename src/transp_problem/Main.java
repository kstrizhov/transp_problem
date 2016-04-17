package transp_problem;

import java.util.ArrayList;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		//DataImporter.createData();

		ArrayList<Mine> producers = new ArrayList<Mine>();
		producers.add(new Mine(1));
		producers.add(new Mine(2));
		producers.add(new Mine(3));
		producers.add(new Mine(4));
		
		ArrayList<ConsumptionPoint> consumers = new ArrayList<ConsumptionPoint>();
		consumers.add(new ConsumptionPoint(5));
		consumers.add(new ConsumptionPoint(1));
		consumers.add(new ConsumptionPoint(2));
		consumers.add(new ConsumptionPoint(2));
		
		int[][] plan = Planner.createBasicPlan(producers, consumers);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++)
				System.out.print(plan[i][j]);
			System.out.println("");
		}
	}
}
