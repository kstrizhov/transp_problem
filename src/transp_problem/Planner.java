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

	protected static class Plan {
		protected Matrix X0;
		protected ArrayList<MatrixElement> basis;

		protected Plan(Matrix plan, ArrayList<MatrixElement> basis) {
			this.X0 = plan;
			this.basis = basis;
		}
	}

	public static Plan createBasicPlan(List<Mine> p, List<ConsumptionPoint> c) {

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

		Matrix X0 = new Matrix(numOfProducers, numOfConsumers, INITIAL_PLAN_MATRIX_VALUE);
		ArrayList<MatrixElement> basis = new ArrayList<MatrixElement>();

		int numOfSteps = numOfProducers + numOfConsumers - 1;

		for (int k = 0, i = 0, j = 0; k < numOfSteps; k++) {

			if (i == numOfProducers || j == numOfConsumers)
				break;

			double production = producers.get(i).getProduction();
			double consumption = consumers.get(j).getConsumption();

			MatrixElement element;

			if (production < consumption) {
				X0.set(i, j, production);
				element = new MatrixElement(i, j, production);
				consumers.get(j).setConsumption(consumption - production);
				producers.get(i).setProduction(0);
				i++;
			} else {
				X0.set(i, j, consumption);
				element = new MatrixElement(i, j, consumption);
				producers.get(i).setProduction(production - consumption);
				consumers.get(j).setConsumption(0);
				j++;
			}

			basis.add(element);
		}

		return new Plan(X0, basis);
	};

	public static void solve(Plan plan, Matrix C0) {

		Matrix X0 = plan.X0;

		printResult(X0, C0);

		PotentialVectorItem p = getPotentialVector(plan.basis, C0);

		Matrix C1 = calculateCostMatrix(p, C0);

		if (checkOnOptimum(C1)) {
			printResult(X0, C0);
			return;
		}

		MatrixElement minC1element = findMinElement(C1);

		HashMap<Integer, MatrixElement> cycle = findCycle(plan, minC1element);

		reallocateSupplies(plan, cycle);

		if (checkBasisForSingularity(plan.basis)) {
			System.err.println("SINGULAR!");
			removeSupplyFromSingularBasis(plan.basis);
		} else
			removeSupplyFromBasis(plan.basis);

		PotentialVectorItem newP = getPotentialVector(plan.basis, C0);

		Matrix newC1 = calculateCostMatrix(newP, C0);

		if (checkOnOptimum(newC1)) {
			printResult(X0, C0);
		} else {
			System.out.println("next step");
			solve(plan, C0);
		}
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
	private final static int VECTOR_COLUMN_NUMBER = 0;

	private final static double Ui_COEFFICIENT = -1;
	private final static double Vj_COEFFICIENT = 1;

	private static MatrixSet getCoeffMatrices(ArrayList<MatrixElement> basis, Matrix C) {
		int numOfRows = C.getRowDimension();
		int numOfColumns = C.getColumnDimension();
		int numOfConditions = numOfRows + numOfColumns;

		Matrix A = new Matrix(numOfConditions, numOfConditions);
		Matrix B = new Matrix(numOfConditions, NUM_OF_COLUMNS_IN_VECTOR);

		A.set(U1_ROW, U1_COLUMN, U1_COEFFICIENT);
		B.set(U1_ROW, U1_COLUMN, U1_VALUE);

		int equationNumber = U1_ROW + 1;

		for (MatrixElement e : basis) {
			int i = e.row;
			int j = e.column;
			A.set(equationNumber, i, Ui_COEFFICIENT);
			A.set(equationNumber, j + numOfRows, Vj_COEFFICIENT);
			double cost = C.get(i, j);
			B.set(equationNumber, VECTOR_COLUMN_NUMBER, cost);
			equationNumber++;
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

	private static PotentialVectorItem getPotentialVector(ArrayList<MatrixElement> basis, Matrix C) {

		int numOfProducers = C.getRowDimension();
		int numOfConsumers = C.getColumnDimension();

		MatrixSet set = getCoeffMatrices(basis, C);
		Matrix P = set.A.solve(set.B);

		return new PotentialVectorItem(P, numOfProducers, numOfConsumers);
	}

	private static Matrix calculateCostMatrix(PotentialVectorItem item, Matrix C) {

		Matrix C1 = C.copy();

		int numOfRows = C1.getRowDimension();
		int numOfColumns = C1.getColumnDimension();
		int vectorColumnNum = VECTOR_COLUMN_NUMBER;

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

	protected static class MatrixElement {
		double value;
		int row;
		int column;
		ElementSign sign;

		private MatrixElement(int row, int column, double value) {
			this.row = row;
			this.column = column;
			this.value = value;
		}

		private MatrixElement(int row, int column, double value, ElementSign sign) {
			this.row = row;
			this.column = column;
			this.value = value;
			this.sign = sign;
		}

		private boolean equals(MatrixElement e) {
			if (this.row == e.row && this.column == e.column && this.value == e.value)
				return true;
			return false;
		}

		private boolean equals(MatrixElement e, boolean checkSign) {
			if (checkSign) {
				if (this.row == e.row && this.column == e.column && this.value == e.value && this.sign == e.sign)
					return true;
			} else if (this.row == e.row && this.column == e.column && this.value == e.value)
				return true;
			return false;
		}

		public void print(boolean printSign) {
			System.out.print("value: " + value + "  row: " + row + "  column: " + column);
			if (printSign)
				System.out.println("  sign: " + sign);
			System.out.println("");
		}
	}

	private static enum StringType {
		ROW, COLUMN
	}

	private static HashMap<Integer, MatrixElement> findCycle(Plan plan, MatrixElement element) {
		Matrix X0 = plan.X0;
		ArrayList<MatrixElement> basis = plan.basis;

		int numOfRows = X0.getRowDimension();
		int numOfColumns = X0.getColumnDimension();

		MatrixElement newBasiselement = new MatrixElement(element.row, element.column,
				X0.get(element.row, element.column));
		basis.add(newBasiselement);

		StringMapSet set = scratchOutRule(plan);

		int initialElementNumber = 0;

		HashMap<Integer, MatrixElement> cycleMap = new HashMap<Integer, MatrixElement>();
		cycleMap.put(initialElementNumber, newBasiselement);

		int cycleElementCount = initialElementNumber;
		boolean cycleIsLocked = false;
		StringType type = StringType.ROW;

		do {
			int element_i = cycleMap.get(cycleElementCount).row;
			int element_j = cycleMap.get(cycleElementCount).column;

			switch (type) {
			case ROW:
				rowLoop: for (int j = 0; j < numOfColumns; j++) {
					if (!set.columnsMap.containsKey(j))
						continue;
					MatrixElement nextElement = new MatrixElement(element_i, j, X0.get(element_i, j));
					for (MatrixElement e : basis)
						if (e.equals(nextElement) && j != element_j) {
							cycleMap.put(++cycleElementCount, nextElement);
							element_j = j;
							type = StringType.COLUMN;
							break rowLoop;
						}
				}
				break;
			case COLUMN:
				columnLoop: for (int i = 0; i < numOfRows; i++) {
					if (!set.rowsMap.containsKey(i))
						continue;
					MatrixElement nextElement = new MatrixElement(i, element_j, X0.get(i, element_j));
					for (MatrixElement e : basis)
						if (e.equals(nextElement) && i != element_i) {
							cycleMap.put(++cycleElementCount, nextElement);
							element_i = i;
							type = StringType.ROW;
							break columnLoop;
						}
				}
				break;
			}

			if (element_i == cycleMap.get(initialElementNumber).row
					&& element_j == cycleMap.get(initialElementNumber).column) {
				cycleMap.remove(cycleElementCount);
				cycleIsLocked = true;
			}
		} while (cycleIsLocked != true);

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

		MatrixElement minSupplyElement = null;

		label: for (MatrixElement e : cycle.values()) {
			switch (e.sign) {
			case PLUS:
				break;
			case MINUS:
				minSupplyElement = e;
				break label;
			}
		}

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

	private static void reallocateSupplies(Plan plan, Map<Integer, MatrixElement> cycle) {
		Matrix X = plan.X0;
		ArrayList<MatrixElement> basis = plan.basis;

		setCycleSigns(cycle);

		double minSupplyValue = getMinimalSupply(cycle);

		for (MatrixElement e : cycle.values()) {
			int i = e.row;
			int j = e.column;
			double value = X.get(i, j);

			switch (e.sign) {
			case PLUS:
				X.set(i, j, value + minSupplyValue);
				for (MatrixElement e1 : basis)
					if (e.equals(e1, false))
						e1.value = value + minSupplyValue;
				break;
			case MINUS:
				X.set(i, j, value - minSupplyValue);
				for (MatrixElement e1 : basis)
					if (e.equals(e1, false))
						e1.value = value - minSupplyValue;
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

	private static StringMapSet scratchOutRule(Plan plan) {
		int numOfRows = plan.X0.getRowDimension();
		int numOfColumns = plan.X0.getColumnDimension();

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
					switch (checkRow(plan, columnsMap, i)) {
					case LEAVE:
						break;
					case SCRATCH_OUT:
						rowsMap.remove(i);
						stringWasScratchedOut = true;
						break;
					}
				}
			}

			for (int j = 0; j < numOfColumns; j++) {
				if (columnsMap.containsKey(j)) {
					switch (checkColumn(plan, rowsMap, j)) {
					case LEAVE:
						break;
					case SCRATCH_OUT:
						columnsMap.remove(j);
						stringWasScratchedOut = true;
						break;
					}
				}
			}
		} while (stringWasScratchedOut == true);

		if (rowsMap.isEmpty() || columnsMap.isEmpty())
			return null;
		else
			return new StringMapSet(rowsMap, columnsMap);
	}

	/* ----- HELP METODS ----- */

	private static void printResult(Matrix X0, Matrix C0) {
		System.out.println("Optimum plan");
		System.out.println("Cost function: " + calculateCostFunction(X0, C0));
		X0.print(X0.getColumnDimension(), 2);
	}

	private enum StringState {
		SCRATCH_OUT, LEAVE
	}

	private static StringState checkRow(Plan plan, Map<Integer, Integer> columnsMap, int rowNumber) {
		Matrix X0 = plan.X0;
		ArrayList<MatrixElement> basis = plan.basis;

		int count = 0;

		for (int j = 0; j < X0.getColumnDimension(); j++) {
			int e_i = rowNumber;
			int e_j = j;
			double e_value = X0.get(e_i, e_j);
			MatrixElement e = new MatrixElement(e_i, e_j, e_value);
			for (MatrixElement element : basis)
				if (element.equals(e) && columnsMap.containsKey(e.column))
					count++;
			if (count >= 2)
				return StringState.LEAVE;
		}

		return StringState.SCRATCH_OUT;
	}

	private static StringState checkColumn(Plan plan, Map<Integer, Integer> rowsMap, int columnNumber) {
		Matrix X0 = plan.X0;
		ArrayList<MatrixElement> basis = plan.basis;

		int count = 0;

		for (int i = 0; i < X0.getRowDimension(); i++) {
			int e_i = i;
			int e_j = columnNumber;
			double e_value = X0.get(e_i, e_j);
			MatrixElement e = new MatrixElement(e_i, e_j, e_value);
			for (MatrixElement element : basis)
				if (element.equals(e) && rowsMap.containsKey(e.row))
					count++;
			if (count >= 2)
				return StringState.LEAVE;
		}

		return StringState.SCRATCH_OUT;
	}

	private static void checkBasicPlan(Plan plan, ArrayList<Mine> producers, ArrayList<ConsumptionPoint> consumers) {
		int numOfRows = plan.X0.getRowDimension();
		int numOfColumns = plan.X0.getColumnDimension();

		double rowSum = 0;
		double columnSum = 0;

		for (int i = 0; i < numOfRows; i++) {
			for (int j = 0; j < numOfColumns; j++) {
				rowSum += plan.X0.get(i, j);
			}
			if (rowSum != producers.get(i).getProduction()) {
				System.out.println(
						"Basic plan is not correct: row " + i + " sum is not equal to producers[" + i + "] production");
				return;
			}
		}

		for (int j = 0; j < numOfColumns; j++) {
			for (int i = 0; i < numOfRows; i++) {
				columnSum += plan.X0.get(i, j);
			}
			if (columnSum != consumers.get(j).getConsumption()) {
				System.out.println("Basic plan is not correct: column " + j + " sum is not equal to consumers[" + j
						+ "] consumption");
				return;
			}
		}

		System.out.println("Basic plan is correct");
	}

	private static boolean checkBasisForSingularity(ArrayList<MatrixElement> basis) {
		int count = 0;
		for (MatrixElement e : basis) {
			if (e.value == 0)
				count++;
			if (count > 1)
				return true;
		}
		return false;
	}

	private static void removeSupplyFromSingularBasis(ArrayList<MatrixElement> basis) {
		ArrayList<MatrixElement> nullElementsList = new ArrayList<MatrixElement>();
		for (MatrixElement e : basis)
			if (e.value == 0)
				nullElementsList.add(e);

		MatrixElement min_row_element = nullElementsList.get(0);

		for (MatrixElement e : nullElementsList)
			if (e.row < min_row_element.row)
				min_row_element = e;

		basis.remove(min_row_element);
	}

	private static void removeSupplyFromBasis(ArrayList<MatrixElement> basis) {
		MatrixElement nullElement = null;
		for (MatrixElement e : basis)
			if (e.value == 0) {
				nullElement = e;
				break;
			}

		basis.remove(nullElement);
	}

	private static MatrixElement findMinElement(Matrix M) {
		int numOfRows = M.getRowDimension();
		int numOfColumns = M.getColumnDimension();

		int min_i = 0;
		int min_j = 0;
		double minElement = M.get(min_i, min_j);

		for (int i = 0; i < numOfRows; i++)
			for (int j = 0; j < numOfColumns; j++)
				if (M.get(i, j) < minElement) {
					minElement = M.get(i, j);
					min_i = i;
					min_j = j;
				}

		return new MatrixElement(min_i, min_j, minElement);
	}
}
