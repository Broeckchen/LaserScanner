
public class ModelParameters {

	private int lazerRange;
	private int angleRange;
	private int angleStep;

	private double zSmall;
	private int cmPerPixel;
	private double l;
	private double b;
	
	public ModelParameters(int lazerRange,int cmPerPixel, int angleRange, int angleStep, double l, double b, double zSmall){
		this.lazerRange = lazerRange;
		this.cmPerPixel = cmPerPixel;
		this.angleRange = angleRange;
		this.angleStep = angleStep;
		this.l = l;
		this.b = b;
		this.zSmall = zSmall;
	}
	
	public int getLaserRange() {
		return lazerRange;
	}
	public void setLaserRange(int lazerRange) {
		this.lazerRange = lazerRange;
	}
	public int getAngleRange() {
		return angleRange;
	}
	public void setAngleRange(int angleRange) {
		this.angleRange = angleRange;
	}
	
	public int getAngleStep() {
		return angleStep;
	}

	public void setAngleStep(int angleStep) {
		this.angleStep = angleStep;
	}

	public int getCmPerPixel() {
		return cmPerPixel;
	}

	public void setCmPerPixel(int cmPerPixel) {
		this.cmPerPixel = cmPerPixel;
	}

	public double getzSmall() {
		return zSmall;
	}

	public void setzSmall(double zSmall) {
		this.zSmall = zSmall;
	}
	public double getL() {
		return l;
	}
	public void setL(double l) {
		this.l = l;
	}
	public double getB() {
		return b;
	}
	public void setB(double b) {
		this.b = b;
	}
	
	
}
