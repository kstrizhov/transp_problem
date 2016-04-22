package transp_problem;

public class ConsumptionPoint {

	private double consumption;

	public ConsumptionPoint(double consumption) {
		this.setConsumption(consumption);
	}

	public double getConsumption() {
		return consumption;
	}

	public void setConsumption(double consumption) {
		this.consumption = consumption;
	}

	public ConsumptionPoint copy() {
		return new ConsumptionPoint(consumption);
	}
}
