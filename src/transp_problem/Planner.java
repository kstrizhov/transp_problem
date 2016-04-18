package transp_problem;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;

public class Planner {

	public static IntMatrix createBasicPlan(List<Mine> producers, List<ConsumptionPoint> consumers) {
		int numOfProducers = producers.size();
		int numOfConsumers = consumers.size();

		IntMatrix planMatrix = new IntMatrix(numOfProducers, numOfConsumers, -1);

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

	public static MatrixSet getCoeffMatrix(Matrix X, Matrix C) {
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
		
		A.print(A.getColumnDimension(), 2);
		B.print(B.getColumnDimension(), 2);

		return new MatrixSet(A, B);
	}

	public static Matrix getPotentialVector(MatrixSet set) throws Exception {
		return solveWithKramer(set.A, set.B);
	}

	/*------Kramer's method-------*/

	private static Matrix solveWithKramer(Matrix A, Matrix B) throws Exception {

		if (A.getRowDimension() != B.getRowDimension())
			throw new Exception("Numbers of rows of A and B are not equal!");

		int numOfEquations = A.getRowDimension();

		Matrix X = new Matrix(numOfEquations, 1);
		ArrayList<Matrix> matrixList = new ArrayList<Matrix>();

		for (int i = 0; i < numOfEquations; i++) {
			Matrix D = A.copy();
			for (int j = 0; j < numOfEquations; j++) {
				D.set(j, i, B.get(j, 0));
			}
			matrixList.add(D);
		}

		for (int i = 0; i < numOfEquations; i++) {
			double solution = matrixList.get(i).det() / A.det();
			X.set(i, 0, solution);
			System.out.println(matrixList.get(i).det());
			int[] rows = {1,2,3,4,5,6};
			int[] cols = {1,2,3,4,5,6};
			System.err.println("DET: " + A.getMatrix(rows,cols).det());
		}

		return X;
	}
}
