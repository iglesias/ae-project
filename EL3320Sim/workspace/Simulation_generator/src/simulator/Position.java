package simulator;

public class Position {
	double trueX;
	double trueY;
	double trueTheta;
	public Position(double trueX, double trueY, double trueTheta) {
		this.trueX = trueX;
		this.trueY = trueY;
		this.trueTheta = trueTheta;
	}
	public double getTrueX() {
		return trueX;
	}
	public void setTrueX(double trueX) {
		this.trueX = trueX;
	}
	public double getTrueY() {
		return trueY;
	}
	public void setTrueY(double trueY) {
		this.trueY = trueY;
	}
	public double getTrueTheta() {
		return trueTheta;
	}
	public void setTrueTheta(double trueTheta) {
		this.trueTheta = trueTheta;
	}
}
