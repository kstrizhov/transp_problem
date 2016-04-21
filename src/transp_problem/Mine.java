package transp_problem;

public class Mine extends Plant {
	
	private double production;
	
	public Mine(double production) {
		this.setProduction(production);
	}

	public double getProduction() {
		return production;
	}

	public void setProduction(double production) {
		this.production = production;
	}

}
