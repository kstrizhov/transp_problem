package transp_problem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import Jama.Matrix;

public class Planner {

	public static double calculateCostFunction(Matrix X, Matrix C) {
		int numOfRows = X.getRowDimension();
		int numOfColumns = X.getColumnDimension();

		double cost = 0;

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				cost += C.get(i, j) * X.get(i, j);

		return cost;
	}

	public static Matrix createBasicPlan(List<Mine> producers, List<ConsumptionPoint> consumers) {
		int numOfProducers = producers.size();
		int numOfConsumers = consumers.size();

		Matrix planMatrix = new Matrix(numOfProducers, numOfConsumers, -1);

		int numOfSteps = numOfProducers + numOfConsumers - 1;

		for (int k = 0, i = 0, j = 0; k < numOfSteps; k++) {

			if (i == numOfProducers || j == numOfConsumers)
				break;

			double production = producers.get(i).getProduction();
			double consumption = consumers.get(j).getConsumption();

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

	public static void solve(Matrix X0, Matrix C0) {

		Planner.PotentialVectorItem p = Planner.getPotentialVector(X0, C0);

		Matrix C1 = Planner.calculateCostMatrix(p, C0);

		if (Planner.checkOnOptimum(C1)) {
			System.out.println("Optimum plan");
			System.out.println("Cost function: " + Planner.calculateCostFunction(X0, C0));
			X0.print(X0.getColumnDimension(), 2);
			return;
		}

		C1.print(C1.getColumnDimension(), 2);

		MatrixElement minC1element = Planner.findMinElement(C1);

		HashMap<Integer, MatrixElement> cycle = Planner.findCycle(X0, minC1element);

		for (Integer i : cycle.keySet()) {
			System.err.print("cycle element N" + i + ": ");
			cycle.get(i).print(true);
			System.err.println("");
		}

		System.out.println("BEFORE");

		X0.print(X0.getColumnDimension(), 2);

		Planner.reallocateSupplies(X0, cycle);

		System.out.println("AFTER");

		X0.print(X0.getColumnDimension(), 2);

		Planner.PotentialVectorItem newP = Planner.getPotentialVector(X0, C0);

		Matrix newC1 = Planner.calculateCostMatrix(newP, C0);

		if (Planner.checkOnOptimum(newC1)) {
			System.out.println("Optimum plan");
			System.out.println("Cost function: " + Planner.calculateCostFunction(X0, C0));
			X0.print(X0.getColumnDimension(), 2);
		} else {
			System.out.println("qqq");
			solve(X0, C0);
		}
	}

	public static boolean isSingular(Matrix X) {
		int numOfRows = X.getRowDimension();
		int numOfColumns = X.getColumnDimension();

		int numOfConditions = numOfRows + numOfColumns;
		int count = 0;

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				if (X.get(i, j) != 0)
					count++;
		if (count != numOfConditions - 1)
			return true;
		return false;
	}

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

		A.set(0, 0, 1); // set +1 before u1
		B.set(0, 0, 0); // set u1 = 0;

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

	enum ElementSign {
		PLUS, MINUS
	}

	public static class MatrixElement {
		double value;
		int row;
		int column;
		ElementSign sign;

		public MatrixElement(double value, int row, int column) {
			this.value = value;
			this.row = row;
			this.column = column;
		}

		public MatrixElement(double value, int row, int column, ElementSign sign) {
			this.value = value;
			this.row = row;
			this.column = column;
			this.sign = sign;
		}

		public void print(boolean printSign) {
			System.err.print("value: " + value + "  row: " + row + "  column: " + column);
			if (printSign)
				System.err.println("  sign: " + sign);
		}
	}

	public static MatrixElement findMinElement(Matrix M) {
		int numOfRows = M.getRowDimension();
		int numOfColumns = M.getColumnDimension();

		double minElement = Double.POSITIVE_INFINITY;
		int min_i = 0;
		int min_j = 0;

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				if (M.get(i, j) < minElement) {
					minElement = M.get(i, j);
					min_i = i;
					min_j = j;
				}

		return new MatrixElement(minElement, min_i, min_j);
	}

	public static HashMap<Integer, MatrixElement> findCycle(Matrix M, MatrixElement element) {

		int numOfRows = M.getRowDimension();
		int numOfColumns = M.getColumnDimension();

		int originalElementRow = element.row;
		int originalElementColumn = element.column;
		double originalElementValue = M.get(originalElementRow, originalElementColumn);

		M.set(element.row, element.column, element.value);

		StringMapSet set = scratchOutRule(M);

		HashMap<Integer, MatrixElement> cycleMap = new HashMap<Integer, MatrixElement>();
		cycleMap.put(0, element);

		int cycleElementCount = 0;
		StringType type = StringType.ROW;

		do {
			int element_i = cycleMap.get(cycleElementCount).row;
			int element_j = cycleMap.get(cycleElementCount).column;

			switch (type) {
			case ROW:
				for (int j = 0; j < numOfColumns; j++) {
					if (!set.columnsMap.containsKey(j))
						continue;
					if (M.get(element_i, j) != 0 && j != element_j) {
						MatrixElement nextElement = new MatrixElement(M.get(element_i, j), element_i, j);
						cycleMap.put(++cycleElementCount, nextElement);
						element_j = j;
						type = StringType.COLUMN;
						break;
					}
				}
				break;
			case COLUMN:
				for (int i = 0; i < numOfRows; i++) {
					if (!set.rowsMap.containsKey(i))
						continue;
					if (M.get(i, element_j) != 0 && i != element_i) {
						MatrixElement nextElement = new MatrixElement(M.get(i, element_j), i, element_j);
						cycleMap.put(++cycleElementCount, nextElement);
						element_i = i;
						type = StringType.ROW;
						break;
					}
				}
				break;
			}
		} while (cycleElementCount != 2 * set.rowsMap.size() - 1);

		M.set(originalElementRow, originalElementColumn, originalElementValue);

		for (Integer key : cycleMap.keySet())
			if (cycleMap.get(key).value < 0)
				cycleMap.get(key).value = Double.POSITIVE_INFINITY;

		setCycleSigns(cycleMap);

		return cycleMap;
	}

	private static void setCycleSigns(Map<Integer, MatrixElement> cycle) {
		for (int i = 0; i < cycle.size(); i++)
			if (i % 2 == 0)
				cycle.get(i).sign = ElementSign.PLUS;
			else
				cycle.get(i).sign = ElementSign.MINUS;
	}

	public static double getMinimalSupply(Map<Integer, MatrixElement> cycle) {

		MatrixElement minSupplyElement = cycle.get(0);

		for (Integer i : cycle.keySet()) {
			switch (cycle.get(i).sign) {
			case PLUS:
				break;
			case MINUS:
				if (cycle.get(i).value < minSupplyElement.value)
					minSupplyElement = cycle.get(i);
				break;
			}
		}

		return minSupplyElement.value;
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

	public static void reallocateSupplies(Matrix X, Map<Integer, MatrixElement> cycle) {

		double minSupplyValue = getMinimalSupply(cycle);

		for (Integer key : cycle.keySet()) {
			int i = cycle.get(key).row;
			int j = cycle.get(key).column;
			double value = X.get(i, j);

			switch (cycle.get(key).sign) {
			case PLUS:
				X.set(i, j, value + minSupplyValue);
				break;
			case MINUS:
				X.set(i, j, value - minSupplyValue);
				break;
			}
		}
	}

	private static class StringMapSet {
		Map<Integer, Integer> rowsMap = new HashMap<Integer, Integer>();
		Map<Integer, Integer> columnsMap = new HashMap<Integer, Integer>();

		private StringMapSet(Map<Integer, Integer> rowsMap, Map<Integer, Integer> columnsMap) {
			this.rowsMap = rowsMap;
			this.columnsMap = columnsMap;
		}
	}

	private static StringMapSet scratchOutRule(Matrix M) {

		int numOfRows = M.getRowDimension();
		int numOfColumns = M.getColumnDimension();

		double[] rows = M.getRowPackedCopy();
		double[] cols = M.getColumnPackedCopy();

		Map<Integer, Integer> rowsMap = new HashMap<Integer, Integer>();
		for (int i = 0; i < numOfRows; i++) {
			rowsMap.put(i, i);
		}

		Map<Integer, Integer> columnsMap = new HashMap<Integer, Integer>();
		for (int i = 0; i < numOfColumns; i++) {
			columnsMap.put(i, i);
		}

		boolean stringWasScratchedOut;

		do {
			stringWasScratchedOut = false;
			for (int i = 0; i < numOfRows; i++) {
				if (rowsMap.containsKey(i)) {
					double[] row = getStringFromPack(rows, numOfColumns, i);
					switch (checkMatrixString(row, columnsMap)) {
					case LEAVE:
						continue;
					case SCRATCH_OUT:
						rowsMap.remove(i);
						stringWasScratchedOut = true;
						continue;
					}
				}
			}

			for (int i = 0; i < numOfColumns; i++) {
				if (columnsMap.containsKey(i)) {
					double[] column = getStringFromPack(cols, numOfRows, i);
					switch (checkMatrixString(column, rowsMap)) {
					case LEAVE:
						continue;
					case SCRATCH_OUT:
						columnsMap.remove(i);
						stringWasScratchedOut = true;
						continue;
					}
				}
			}
		} while (stringWasScratchedOut == true);

		if (rowsMap.isEmpty() || columnsMap.isEmpty())
			return null;
		else
			return new StringMapSet(rowsMap, columnsMap);
	}

	enum StringState {
		SCRATCH_OUT, LEAVE
	}

	private static StringState checkMatrixString(double[] string, Map<Integer, Integer> stringElementsMap) {
		int count = 0;

		for (int i = 0; i < string.length; i++) {
			if (!stringElementsMap.containsKey(i))
				continue;
			if (string[i] != 0)
				count++;
			if (count >= 2)
				return StringState.LEAVE;
		}

		return StringState.SCRATCH_OUT;
	}

	public static enum StringType {
		ROW, COLUMN
	}

	public static Matrix scratchOutStrings(Matrix M, int num, StringType type) {
		int numOfRows = M.getRowDimension();
		int numOfColumns = M.getColumnDimension();
		List<Integer> numList = new ArrayList<Integer>();
		Matrix subM = null;

		switch (type) {
		case ROW:
			for (int i = 0; i < numOfRows; i++)
				numList.add(i);
			numList.remove(num);
			int[] rowNums = new int[numList.size()];
			for (int i = 0; i < numList.size(); i++)
				rowNums[i] = numList.get(i);
			subM = M.getMatrix(rowNums, 0, numOfColumns - 1);
			break;
		case COLUMN:
			for (int i = 0; i < numOfColumns; i++)
				numList.add(i);
			numList.remove(num);
			int[] columnNums = new int[numList.size()];
			for (int i = 0; i < numList.size(); i++)
				columnNums[i] = numList.get(i);
			subM = M.getMatrix(0, numOfRows - 1, columnNums);
			break;
		}

		return subM;
	}

	private static double[] getStringFromPack(double[] strings, int stringLength, int stringNum) {
		double[] string = new double[stringLength];
		for (int i = 0; i < stringLength; i++) {
			string[i] = strings[stringNum * stringLength + i];
		}

		return string;
	}
}
