package transp_problem;

import java.util.ArrayList;

import Jama.Matrix;

public class Main {

	public static void main(String[] args) {
		DataImporter.createData();

		ArrayList<Mine> producers = new ArrayList<Mine>();
		producers.add(new Mine(11));
		producers.add(new Mine(11));
		producers.add(new Mine(8));
		producers.add(new Mine(4));

		ArrayList<ConsumptionPoint> consumers = new ArrayList<ConsumptionPoint>();
		consumers.add(new ConsumptionPoint(5));
		consumers.add(new ConsumptionPoint(9));
		consumers.add(new ConsumptionPoint(9));
		consumers.add(new ConsumptionPoint(7));

		IntMatrix plan = Planner.createBasicPlan(producers, consumers);
		plan.print();

		double[] x = { 5, 6, 0, 0, 0, 3, 8, 0, 0, 0, 1, 7 };
		double[] c1 = { 7, 8, 5, 3, 2, 4, 5, 9, 6, 3, 1, 2 };

		Matrix X = (new Matrix(x, 4)).transpose();
		Matrix C1 = (new Matrix(c1, 4)).transpose();

		Planner.MatrixSet set = Planner.getCoeffMatrix(X, C1);

		Matrix potentials = null;
		try {
			potentials = Planner.getPotentialVector(set);
		} catch (Exception e) {
			e.printStackTrace();
		}

		potentials.print(potentials.getColumnDimension(), 2);
	}
}
