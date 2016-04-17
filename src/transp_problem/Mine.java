package transp_problem;

public class Mine extends Plant {
	
	private int production;
	
	public Mine(int production) {
		this.setProduction(production);
	}

	public int getProduction() {
		return production;
	}

	public void setProduction(int production) {
		this.production = production;
	}

}
