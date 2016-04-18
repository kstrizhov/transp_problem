package transp_problem;

import java.util.List;

import Jama.Matrix;

public class Planner {

	public static Matrix createBasicPlan(List<Mine> producers, List<ConsumptionPoint> consumers) {
		int numOfProducers = producers.size();
		int numOfConsumers = consumers.size();

		Matrix planMatrix = new Matrix(numOfProducers, numOfConsumers, -1);

		int numOfSteps = numOfProducers + numOfConsumers - 1;

		for (int k = 0, i = 0, j = 0; k < numOfSteps; k++) {

			if (i == numOfProducers || j == numOfConsumers)
				break;

			int production = producers.get(i).getProduction();
			int consumption = consumers.get(j).getConsumption();

			if (production < consumption) {
				planMatrix.set(i, j, production);
				consumers.get(j).setConsumption(consumption - production);
				producers.get(i).setProduction(0);
				for (int t = j + 1; t < numOfConsumers; t++)
					planMatrix.set(i, t, 0);
				i++;
			} else {
				planMatrix.set(i, j, consumption);
				producers.get(i).setProduction(production - consumption);
				consumers.get(j).setConsumption(0);
				for (int t = i + 1; t < numOfProducers; t++)
					planMatrix.set(t, j, 0);
				j++;
			}
		}

		return planMatrix;
	};

	public static class MatrixSet {
		private Matrix A;
		private Matrix B;

		private MatrixSet(Matrix A, Matrix B) {
			this.A = A;
			this.B = B;
		}
	}

	private static MatrixSet getCoeffMatrices(Matrix X, Matrix C) {
		int numOfRows = X.getRowDimension();
		int numOfColumns = X.getColumnDimension();
		int numOfConditions = numOfRows + numOfColumns;

		Matrix A = new Matrix(numOfConditions, numOfConditions);
		Matrix B = new Matrix(numOfConditions, 1);

		A.set(0, 0, 1);
		B.set(0, 0, 0);

		int k = 1;

		for (int i = 0; i < X.getRowDimension(); i++)
			for (int j = 0; j < X.getColumnDimension(); j++)
				if (X.get(i, j) != 0) {
					double uCoeff = -1;
					double vCoeff = 1;
					A.set(k, i, uCoeff);
					A.set(k, j + numOfRows, vCoeff);
					double cost = C.get(i, j);
					B.set(k, 0, cost);
					k++;
				}

		return new MatrixSet(A, B);
	}

	public static class PotentialVectorItem {

		private Matrix P;
		private int numOfProducers;
		private int numOfConsumers;

		public PotentialVectorItem(Matrix P, int numOfProducers, int numOfConsumers) {
			this.P = P;
			this.numOfProducers = numOfProducers;
			this.numOfConsumers = numOfConsumers;
		}
	}

	public static PotentialVectorItem getPotentialVector(Matrix X, Matrix C) {

		int numOfProducers = X.getRowDimension();
		int numOfConsumers = X.getColumnDimension();

		MatrixSet set = getCoeffMatrices(X, C);
		Matrix P = set.A.solve(set.B);

		return new PotentialVectorItem(P, numOfProducers, numOfConsumers);
	}

	public static Matrix calculateCostMatrix(PotentialVectorItem item, Matrix C) {

		Matrix C1 = C.copy();

		int numOfRows = C1.getRowDimension();
		int numOfColumns = C1.getColumnDimension();
		int vectorColumnNum = item.P.getColumnDimension() - 1;

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++) {
				double costValue = C1.get(i, j);
				double v_j = item.P.get(item.numOfProducers + j, vectorColumnNum);
				double u_i = item.P.get(i, vectorColumnNum);
				double newCostValue = costValue - (v_j - u_i);
				C1.set(i, j, newCostValue);
			}

		return C1;
	}
	
	private static class MatrixElement {
		double value;
		int row;
		int column;
		
		private MatrixElement(double value, int row, int column) {
			this.value = value;
			this.row = row;
			this.column = column;
		}
	}
	
	public static MatrixElement findMinElement(Matrix M) {
		int numOfRows = M.getRowDimension();
		int numOfColumns = M.getColumnDimension();
		
		double minElement = Double.POSITIVE_INFINITY;
		int min_i = 0;
		int min_j = 0;
		
		MatrixElement element = new MatrixElement(minElement, min_i, min_j);
		
		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				if (M.get(i, j) < minElement) {
					minElement = M.get(i, j);
					min_i = i;
					min_j = j;
				}
		
		return element;
	}
	
	public static boolean checkOnOptimum(Matrix C) {
		int numOfRows = C.getRowDimension();
		int numOfColumns = C.getColumnDimension();
		
		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				if (C.get(i, j) < 0)
					return false;
		
		return true;
	}
}
