package transp_problem;

import java.util.List;

public class Planner {

	public static int[][] createBasicPlan(List<Mine> producers, List<ConsumptionPoint> consumers) {
		int numOfProducers = producers.size();
		int numOfConsumers = consumers.size();

		int[][] planMatrix = new int[numOfProducers][numOfConsumers];
		for (int i = 0; i < numOfProducers; i++)
			for (int j = 0; j < numOfConsumers; j++)
				planMatrix[i][j] = -1;

		int numOfSteps = numOfProducers + numOfConsumers - 1;

		for (int k = 0, i = 0, j = 0; k < numOfSteps; k++) {

			if (i == numOfProducers || j == numOfConsumers)
				break;

			int production = producers.get(i).getProduction();
			int consumption = consumers.get(j).getConsumption();

			if (production < consumption) {
				planMatrix[i][j] = production;
				consumers.get(j).setConsumption(consumption-production);
				producers.get(i).setProduction(0);
				for (int t = j + 1; t < numOfConsumers; t++)
					planMatrix[i][t] = 0;
				i++;
			} else {
				planMatrix[i][j] = consumption;
				producers.get(i).setProduction(production - consumption);
				consumers.get(j).setConsumption(0);
				for (int t = i + 1; t < numOfProducers; t++)
					planMatrix[t][j] = 0;
				j++;
			}
		}

		return planMatrix;
	};

	private void optimizeBasicPlan() {

		// ...

	};
}
