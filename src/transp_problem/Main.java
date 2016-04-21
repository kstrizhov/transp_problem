package transp_problem;

import java.util.ArrayList;

import Jama.Matrix;

public class Main {

	public static void main(String[] args) {
		ArrayList<Mine> producers = new ArrayList<Mine>();
		producers.add(new Mine(11));
		producers.add(new Mine(11));
		producers.add(new Mine(8));

		ArrayList<ConsumptionPoint> consumers = new ArrayList<ConsumptionPoint>();
		consumers.add(new ConsumptionPoint(5));
		consumers.add(new ConsumptionPoint(9));
		consumers.add(new ConsumptionPoint(9));
		consumers.add(new ConsumptionPoint(7));

		Matrix X0 = Planner.createBasicPlan(producers, consumers);

		double[] c0 = { 7, 8, 5, 3, 2, 4, 5, 9, 6, 3, 1, 2 };

		Matrix C0 = (new Matrix(c0, 4)).transpose();

		Planner.solve(X0, C0);
	}
}
