package transp_problem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import Jama.Matrix;

public class Planner {

	private static double calculateCostFunction(Matrix X, Matrix C) {
		int numOfRows = X.getRowDimension();
		int numOfColumns = X.getColumnDimension();

		double cost = 0;

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				cost += C.get(i, j) * X.get(i, j);

		return cost;
	}

	private final static double INITIAL_PLAN_MATRIX_VALUE = 0;

	public static Matrix createBasicPlan(List<Mine> p, List<ConsumptionPoint> c) {

		List<Mine> producers = new ArrayList<Mine>();
		for (int i = 0; i < p.size(); i++) {
			producers.add(p.get(i).copy());
		}
		List<ConsumptionPoint> consumers = new ArrayList<ConsumptionPoint>();
		for (int i = 0; i < c.size(); i++) {
			consumers.add(c.get(i).copy());
		}

		int numOfProducers = producers.size();
		int numOfConsumers = consumers.size();

		Matrix planMatrix = new Matrix(numOfProducers, numOfConsumers, INITIAL_PLAN_MATRIX_VALUE);

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
				i++;
			} else {
				planMatrix.set(i, j, consumption);
				producers.get(i).setProduction(production - consumption);
				consumers.get(j).setConsumption(0);
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

	private static boolean isSingular(Matrix X) {
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

	private static class MatrixSet {
		private Matrix A;
		private Matrix B;

		private MatrixSet(Matrix A, Matrix B) {
			this.A = A;
			this.B = B;
		}
	}
	
	private final static int U1_ROW = 0;
	private final static int U1_COLUMN = 0;
	private final static double U1_COEFFICIENT = 1;
	private final static double U1_VALUE = 0;
	
	private final static int NUM_OF_COLUMNS_IN_VECTOR = 1;

	private static MatrixSet getCoeffMatrices(Matrix X, Matrix C) {
		int numOfRows = X.getRowDimension();
		int numOfColumns = X.getColumnDimension();
		int numOfConditions = numOfRows + numOfColumns;

		Matrix A = new Matrix(numOfConditions, numOfConditions);
		Matrix B = new Matrix(numOfConditions, NUM_OF_COLUMNS_IN_VECTOR);

		A.set(U1_ROW, U1_COLUMN, U1_COEFFICIENT);
		B.set(U1_ROW, U1_COLUMN, U1_VALUE);

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

	private static class PotentialVectorItem {

		private Matrix P;
		private int numOfProducers;
		private int numOfConsumers;

		private PotentialVectorItem(Matrix P, int numOfProducers, int numOfConsumers) {
			this.P = P;
			this.numOfProducers = numOfProducers;
			this.numOfConsumers = numOfConsumers;
		}
	}

	private static PotentialVectorItem getPotentialVector(Matrix X, Matrix C) {

		int numOfProducers = X.getRowDimension();
		int numOfConsumers = X.getColumnDimension();

		MatrixSet set = getCoeffMatrices(X, C);
		Matrix P = set.A.solve(set.B);

		return new PotentialVectorItem(P, numOfProducers, numOfConsumers);
	}

	private static Matrix calculateCostMatrix(PotentialVectorItem item, Matrix C) {

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

	private enum ElementSign {
		PLUS, MINUS
	}

	private static class MatrixElement {
		double value;
		int row;
		int column;
		ElementSign sign;

		private MatrixElement(double value, int row, int column) {
			this.value = value;
			this.row = row;
			this.column = column;
		}

		private MatrixElement(double value, int row, int column, ElementSign sign) {
			this.value = value;
			this.row = row;
			this.column = column;
			this.sign = sign;
		}

		private void print(boolean printSign) {
			System.err.print("value: " + value + "  row: " + row + "  column: " + column);
			if (printSign)
				System.err.println("  sign: " + sign);
		}
	}

	private static MatrixElement findMinElement(Matrix M) {
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

	private static HashMap<Integer, MatrixElement> findCycle(Matrix M, MatrixElement element) {

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

		for (MatrixElement e : cycleMap.values())
			if (e.value < 0)
				e.value = Double.POSITIVE_INFINITY;

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

	private static double getMinimalSupply(Map<Integer, MatrixElement> cycle) {

		MatrixElement minSupplyElement = cycle.get(0);

		for (MatrixElement e : cycle.values()) {
			switch (e.sign) {
			case PLUS:
				break;
			case MINUS:
				if (e.value < minSupplyElement.value)
					minSupplyElement = e;
				break;
			}
		}

		return minSupplyElement.value;
	}

	private static boolean checkOnOptimum(Matrix C) {
		int numOfRows = C.getRowDimension();
		int numOfColumns = C.getColumnDimension();

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				if (C.get(i, j) < 0)
					return false;

		return true;
	}

	private static void reallocateSupplies(Matrix X, Map<Integer, MatrixElement> cycle) {

		double minSupplyValue = getMinimalSupply(cycle);

		for (MatrixElement e : cycle.values()) {
			int i = e.row;
			int j = e.column;
			double value = X.get(i, j);

			switch (e.sign) {
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

	private enum StringState {
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

	private static enum StringType {
		ROW, COLUMN
	}

	private static Matrix scratchOutStrings(Matrix M, int num, StringType type) {
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
