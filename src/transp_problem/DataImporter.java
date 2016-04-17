package transp_problem;

import java.util.ArrayList;
import java.util.List;

public class DataImporter {

	public static List<Mine> producers = new ArrayList<Mine>();
	public static List<ConsumptionPoint> consumers = new ArrayList<ConsumptionPoint>();

	public static void createData() {
		for (int i = 0; i < 3; i++)
			producers.add(new Mine(i + 1));
		for (int i = 0; i < 3; i++)
			consumers.add(new ConsumptionPoint(3 - i));

		for (Mine mine : producers)
			System.err.println(mine.getProduction());
		for (ConsumptionPoint point : consumers)
			System.err.println(point.getConsumption());
	}
}
