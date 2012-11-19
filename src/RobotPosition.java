
public class RobotPosition {
/*
 * This class describes the position of the
 * robot in axis x, y and its angle  theta.
 * Theta is defined in degrees [0, 360)
 */
	private int x;
	private int y;
	private int theta;
	
	public RobotPosition(int x, int y, int theta){
		this.x = x;
		this.y = y;
		this.theta = theta;
	}
	
	public int getX() {
		return x;
	}
	public void setX(int x) {
		this.x = x;
	}
	public int getY() {
		return y;
	}
	public void setY(int y) {
		this.y = y;
	}
	public int getTheta() {
		return theta;
	}
	public void setTheta(int theta) {
		this.theta = theta;
	}
	
	
}
