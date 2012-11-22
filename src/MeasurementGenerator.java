import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Random;

import javax.imageio.ImageIO;

import org.apache.commons.math3.util.FastMath;

public class MeasurementGenerator {

	protected BufferedImage map; // the given map

	private double[] measurements; // the computed measurements
	private double[] realMeasurements; // the computed measurements
	protected RobotPosition position; // the robot position

	private ModelParameters parameters; // sensor model parameters

	private BufferedImage picture;

	public MeasurementGenerator() {
	}

	public MeasurementGenerator(File mapfile, BufferedImage picture,
			BufferedImage map, RobotPosition pos, ModelParameters param)
			throws IOException {
		this.picture = ImageIO.read(mapfile);
		this.map = map;
		this.position = pos;
		this.parameters = param;
		this.measurements = new double[((2 * parameters.getAngleRange()) / parameters
				.getAngleStep()) + 1];
		this.realMeasurements = new double[((2 * parameters.getAngleRange()) / parameters
				.getAngleStep()) + 1];
		for (int i = 0; i < realMeasurements.length; i++)
			realMeasurements[i] = -1;
		// picture.setRGB(position.getX(), position.getY(),
		// Color.BLUE.getRGB());
	}

	public BufferedImage getMap() {
		return map;
	}

	public void setMap(BufferedImage map) {
		this.map = map;
	}

	public RobotPosition getPosition() {
		return position;
	}

	public void setPosition(RobotPosition position) {
		this.position = position;
	}

	public ModelParameters getParameters() {
		return parameters;
	}

	public void setParameters(ModelParameters parameters) {
		this.parameters = parameters;
	}

	public BufferedImage getPicture() {
		return picture;
	}

	public double[] laserScanMeasurements() {

		exploreLaserSpace(getLaserSpace());
		map.setRGB(position.getX(), position.getY(), Color.BLUE.getRGB());
		for (int i = 0; i < realMeasurements.length; i++) {
			if (realMeasurements[i] < 0)
				realMeasurements[i] = -1;

			measurements[i] = generateLaserMeasurement(realMeasurements[i]);
			int x = (int) (Math.cos(mapMatrixPositionToAngle(i)
					+ position.getTheta()) * measurements[i]);
			int y = (int) (Math.sin(mapMatrixPositionToAngle(i)
					+ position.getTheta()) * measurements[i]);
			if (x >= 0 && x < picture.getWidth() && y >= 0
					&& y < picture.getHeight())
				picture.setRGB(x, y, Color.GREEN.getRGB());

		}

		return measurements;
	}

	public double[] computeMeasurements() {

		int[] xy = new int[2], obstacle = new int[2];
		int angle, point = 0;

		for (int f = -parameters.getAngleRange(); f < parameters
				.getAngleRange() + 1; f += parameters.getAngleStep()) {
			angle = (int) wrapTo360(position.getTheta() + f);
			if (angle == 90 || angle == 270) {
				xy = extremeXYVertical(angle);
				obstacle = getFirstObstaclePositionVertical(xy[0], xy[1]); // expected
																			// measurement

			} else {
				xy = extremeXY(angle);

				obstacle = getFirstObstaclePosition(xy[0], xy[1],
						Math.tan(angle)); // expected measurement

			}
			if (obstacle[0] >= 0) { // if there is an obstacle we generate a
									// measurement for it by giving its distance
									// as input
				realMeasurements[point] = distance(obstacle[0], obstacle[1]);
				measurements[point] = generateLaserMeasurement(distance(
						obstacle[0], obstacle[1]));
				point++;
			} else { // if no obstacle is found we give an expected value larger
						// than our range
				realMeasurements[point] = 0;
				measurements[point] = generateLaserMeasurement(0.0);
				point++;
			}
		}

		return measurements;
	}

	public double[] getRealMeasurements() {
		return realMeasurements;
	}

	/*
	 * This function generates a measurement based on the expected measurement.
	 * First we calculate a measurement and it's probability, then we check if
	 * this pair of measurement and probability belongs to our probability
	 * density function. If it belongs there we return our measurement, else we
	 * try recursively until it does.
	 */

	private double generateLaserMeasurement(double zExp) {

		double prob;
		int z;

		z = randomMeasurement();
		prob = randomMeasurementProbability();

		double[] probalitiesPerMesurent = probabilityDensityFunction(zExp);

		double probOfZ;
		if (z - 1 >= 0 && z + 1 < probalitiesPerMesurent.length)
			probOfZ = 3 * ((probalitiesPerMesurent[z - 1] + probalitiesPerMesurent[z + 1]) / 2);
		else if (z - 1 < 0) {
			probOfZ = 2 * ((probalitiesPerMesurent[z] + probalitiesPerMesurent[z + 1]) / 2);
		} else {
			probOfZ = 2 * ((probalitiesPerMesurent[z - 1] + probalitiesPerMesurent[z]) / 2);
		}
		if (probOfZ >= prob)
			return (z * parameters.getCmPerPixel() * 0.01);
		else
			return generateLaserMeasurement(zExp);
	}

	private int[][] getLaserSpace() {

		int minX, maxX, minY, maxY;

		minX = (int) (position.getX() - parameters.getLaserRange()
				/ (parameters.getCmPerPixel() * 0.01));
		maxX = (int) (position.getX() + parameters.getLaserRange()
				/ (parameters.getCmPerPixel() * 0.01));
		minY = (int) (position.getY() - parameters.getLaserRange()
				/ (parameters.getCmPerPixel() * 0.01));
		maxY = (int) (position.getY() + parameters.getLaserRange()
				/ (parameters.getCmPerPixel() * 0.01));

		if (maxX >= map.getWidth())
			maxX = map.getWidth() - 1;

		if (minX < 0)
			minX = 0;

		if (maxY >= map.getHeight())
			maxY = map.getHeight() - 1;

		if (minY < 0)
			minY = 0;

		int[][] extremes = new int[2][2];

		extremes[0][0] = minX;
		extremes[0][1] = minY;
		extremes[1][0] = maxX;
		extremes[1][1] = maxY;

		return extremes;
	}

	private void exploreLaserSpace(int[][] extremes) {

		// explore top right quadrant
		int angle, laserAngle;
		for (int i = position.getX() - 1; i >= extremes[0][0]; i--) {
			for (int j = position.getY() - 1; j >= extremes[0][1]; j--) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {

					laserAngle = calculateAngle(i, j);

					if (Math.abs(laserAngle) <= 135) {
						laserAngle = mapAngleToMatrixPosition(laserAngle);
						if (distance(i, j) < 10.001
								&& (realMeasurements[laserAngle] < 0 || realMeasurements[laserAngle] > distance(
										i, j))) {
							realMeasurements[laserAngle] = distance(i, j);
							picture.setRGB(i, j, Color.MAGENTA.getRGB());
						}
					}
				}
			}
		}

		// explore top left quadrant
		for (int i = position.getX() + 1; i <= extremes[1][0]; i++) {
			for (int j = position.getY() - 1; j >= extremes[0][1]; j--) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {

					laserAngle = calculateAngle(i, j);

					if (Math.abs(laserAngle) <= 135) {
						laserAngle = mapAngleToMatrixPosition(laserAngle);
						if (distance(i, j) < 10.001
								&& (realMeasurements[laserAngle] < 0 || realMeasurements[laserAngle] > distance(
										i, j))) {
							realMeasurements[laserAngle] = distance(i, j);
							picture.setRGB(i, j, Color.MAGENTA.getRGB());
						}
					}

				}
			}
		}

		// explore bottom left quadrant
		for (int i = position.getX() + 1; i <= extremes[1][0]; i++) {
			for (int j = position.getY() + 1; j <= extremes[1][1]; j++) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {

					laserAngle = calculateAngle(i, j);
					if (Math.abs(laserAngle) <= 135) {
						laserAngle = mapAngleToMatrixPosition(laserAngle);
						if (distance(i, j) < 10.001
								&& (realMeasurements[laserAngle] < 0 || realMeasurements[laserAngle] > distance(
										i, j))) {
							realMeasurements[laserAngle] = distance(i, j);
							picture.setRGB(i, j, Color.MAGENTA.getRGB());
						}
					}

				}
			}
		}

		// explore bottom right quadrant
		for (int i = position.getX() - 1; i >= extremes[0][0]; i--) {
			for (int j = position.getY() + 1; j <= extremes[1][1]; j++) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {

					laserAngle = calculateAngle(i, j);

					if (Math.abs(laserAngle) <= 135) {
						laserAngle = mapAngleToMatrixPosition(laserAngle);
						if (distance(i, j) < 10.001
								&& (realMeasurements[laserAngle] < 0 || realMeasurements[laserAngle] > distance(
										i, j))) {
							realMeasurements[laserAngle] = distance(i, j);
							picture.setRGB(i, j, Color.MAGENTA.getRGB());
						}
					}

				}
			}
		}
	}

	private int calculateAngle(int xObstacle, int yObstacle) {

		int angle = (int) Math
				.toDegrees(Math.atan2(
						(Math.cos(position.getTheta())
								* (xObstacle - position.getX()) - Math
								.sin(position.getTheta())
								* (yObstacle - position.getY())), Math
								.sin(position.getTheta()
										* (yObstacle - position.getY())
										+ Math.cos(position.getTheta())
										* (xObstacle - position.getX()))));

		return angle;
	}

	/*
	 * This functions returns the extreme point of the laser for the given angle
	 * and the RobotPosition. This function is called when the laser angle
	 * regarding our Cartesian system is not 90 or 270 degrees. We have separate
	 * cases regarding the angle value because, on 90 and 270 degrees the
	 * tangent of the angle can not be defined. This function returns only
	 * points that belong to the map. If an extreme value is out of the map
	 * boundaries, the function returns the "larger" point that belongs both to
	 * the map and the laser ray.
	 */
	private int[] extremeXY(int angle) {

		int newXY[] = new int[2];
		double tan = Math.tan(Math.toRadians(angle)); // we have to choose
														// between the 2
														// possible values for
														// further explanation
														// check the report
		int r = (int) (parameters.getLaserRange() / (parameters.getCmPerPixel() * 0.01));

		int x1 = (int) (Math.round(Math.sqrt((Math.pow(r, 2) / (1 + Math.pow(
				tan, 2))))) + position.getX());
		int x2 = (int) -Math.round(Math.sqrt((Math.pow(r, 2) / (Math
				.pow(tan, 2) + 1)))) + position.getX();
		newXY[0] = chooseX(angle, x1, x2);

		if (newXY[0] > map.getWidth() - 1)
			newXY[0] = map.getWidth() - 1;
		else if (newXY[0] < 0)
			newXY[0] = 0;

		newXY[1] = (int) Math.round(tan * (newXY[0] - position.getX())
				+ position.getY());

		if (newXY[1] > map.getHeight() - 1) {
			newXY[1] = map.getHeight() - 1;
			newXY[0] = (int) (((newXY[1] - position.getY()) / tan) + position
					.getX());
		} else if (newXY[1] < 0) {
			newXY[1] = 0;
			newXY[0] = (int) (((newXY[1] - position.getY()) / tan) + position
					.getX());
		}

		picture.setRGB(newXY[0], newXY[1], Color.PINK.getRGB());
		return newXY;
	}

	/*
	 * This functions returns the extreme point of the laser for the given angle
	 * and the RobotPosition. This function is called when the laser angle
	 * regarding our Cartesian system is 90 or 270 degrees. We have separate
	 * cases regarding the angle value because, on 90 and 270 degrees the
	 * tangent of the angle can not be defined. This function returns only
	 * points that belong to the map. If an extreme value is out of the map
	 * boundaries, the function returns the "larger" point that is in the map
	 * and belongs to the laser ray.
	 */
	private int[] extremeXYVertical(int angle) {
		int newXY[] = new int[2];

		newXY[0] = position.getX();
		newXY[1] = chooseY(angle,
				(int) (position.getY() - parameters.getLaserRange()
						/ (parameters.getCmPerPixel() * 0.01)),
				(int) (position.getY() + parameters.getLaserRange()
						/ (parameters.getCmPerPixel() * 0.01)));

		if (newXY[1] >= map.getHeight())
			newXY[1] = map.getHeight() - 1;
		else if (newXY[1] < 0)
			newXY[1] = 0;
		picture.setRGB(newXY[0], newXY[1], Color.PINK.getRGB());
		return newXY;
	}

	/*
	 * This function gets as input the extreme point of this laser ray and
	 * searches for a black pixel that belongs to the laser ray. This function
	 * is called only if the angle of the ray regarding our Cartesian system is
	 * 90 or 270 degrees. Hence, we search only in y axis. This function returns
	 * the point where the first obstacle lies. If no obstacle is found then it
	 * returns (-1, -1).
	 */
	private int[] getFirstObstaclePositionVertical(int x, int y) {

		int sign;
		int[] point = new int[2];
		if (y <= position.getY()) {
			sign = -1;
		} else {
			sign = 1;
		}

		int curr = position.getY();
		while (Math.abs(y - curr + 1) != 0 && curr >= 0) {
			if (map.getRGB(x, curr) == Color.BLACK.getRGB()) {
				point[0] = x;
				point[1] = curr;
				picture.setRGB(x, curr, Color.GRAY.getRGB());

				return point;
			}
			picture.setRGB(x, curr, Color.GREEN.getRGB());
			curr += sign;
		}
		point[0] = -1;
		point[1] = -1;
		return point;
	}

	/*
	 * This function gets as input the extreme point of this laser ray and the
	 * tangent of the laser ray and searches for a black pixel that belongs to
	 * the laser ray. This function is called if the angle of the ray regarding
	 * our Cartesian system is different from 90 or 270 degrees. Hence, we
	 * search on the line that describes the laser ray. This function returns
	 * the point where the first obstacle lies. If no obstacle is found then it
	 * returns (-1, -1).
	 */
	private int[] getFirstObstaclePosition(int x, int y, double tan) {

		double b = position.getY() - tan * position.getX();
		int[] point = new int[2];

		int h = 0;
		int sign, minX, maxX;
		if (y <= position.getY()) {
			sign = -1;
		} else {
			sign = 1;
		}
		if (x < position.getX()) {
			minX = x;
			maxX = position.getX();
		} else {
			minX = position.getX();
			maxX = x;
		}

		int curr = position.getY();
		while (Math.abs(y - curr) >= 1) {
			h = (int) (Math.round((position.getY() - curr) / tan) + position
					.getX());

			if (h < maxX + 1 && h > minX - 1) {
				if (map.getRGB(h, curr) == Color.BLACK.getRGB()) {
					point[0] = h;
					point[1] = curr;
					picture.setRGB(curr, h, Color.GRAY.getRGB());
					System.out.println("Obstacle in " + h + " " + curr
							+ " with x " + x + " y " + y);
					return point;
				}
				picture.setRGB(h, curr, Color.GREEN.getRGB());
			}
			curr += sign;
		}
		if (map.getRGB(x, y) != Color.BLACK.getRGB()) {
			point[0] = -1;
			point[1] = -1;
		} else {
			point[0] = x;
			point[1] = y;
		}
		return point;
	}

	/*
	 * this function wraps the input angle to 360 degrees
	 */
	private int wrapTo360(int angle) {
		while (angle < 0)
			angle += 360;
		return angle % 360;
	}

	/*
	 * This function returns the x that belongs in the angle's corresponding
	 * quarter
	 */
	private int chooseX(int angle, int x1, int x2) {

		if ((angle >= 0 && angle < 90) || (angle >= 270 && angle < 360)) {
			if (x1 >= position.getX())
				return x1;
			if (x2 >= position.getX())
				return x2;
		} else if ((angle >= 90 && angle < 180)
				|| (angle >= 180 && angle < 270)) {
			if (x1 < position.getX())
				return x1;
			if (x2 < position.getX())
				return x2;
		}
		return position.getX();
	}

	/*
	 * This function returns the y that belongs in the angle's corresponding
	 * quarter
	 */
	private int chooseY(int angle, int y1, int y2) {

		if (angle >= 0 && angle < 180) {
			if (y1 >= position.getY())
				return y1;
			if (y2 >= position.getY())
				return y2;
		} else if ((angle >= 180 && angle < 360)) {
			if (y1 <= position.getY())
				return y1;
			if (y2 <= position.getY())
				return y2;
		}
		return position.getY();
	}

	/*
	 * This function returns the distance in meters between the robot's position
	 * and the input position (x,y).
	 */
	private double distance(int x, int y) {
		return Math.sqrt(Math.pow(position.getX() - x, 2)
				+ Math.pow(position.getY() - y, 2))
				* parameters.getCmPerPixel() * 0.01;
	}

	/*
	 * This function calculates the probability for the given parameters h, a1,
	 * a2, a3, a4 and the measurements z and zExp. zExp is the expected
	 * measurement for the current robot position and the angle of the laser ray
	 * and it is calculated.
	 */

	private double probability(double h, double a1, double a2, double a3,
			double a4, double z, double zExp) {

		return h
				* (a1 * hitProb(z, zExp) + a2 * unexpProb(z, zExp) + a3
						* randProb() + a4 * maxProb(z));
	}

	private double probabilityWithNoObservation(double h, double a3, double a4,
			double z) {

		return h * (a3 * randProb() + a4 * maxProb(z));
	}

	/*
	 * This function returns the probability of having a measurement z while
	 * expecting a measurement zExp normalized by h.
	 */

	private double hitProb(double z, double zExp) {
		return (1 / (Math.sqrt(2 * Math.PI * parameters.getB())) * Math
				.exp(-(0.5) * ((Math.pow(z - zExp, 2)) / parameters.getB())));
	}

	/*
	 * This functions returns the probability of having a measurement z while
	 * expecting a measurement zExp and an unknown obstacle stands before the
	 * expecting obstacle.
	 */

	private double unexpProb(double z, double zExp) {

		if (z < zExp)
			return parameters.getL() * Math.exp(-parameters.getL() * z);
		return 0.0;
	}

	/*
	 * This function returns the probability for a random measurement.
	 */
	private double randProb() {
		return (Math.pow(parameters.getLaserRange(), -1));
	}

	/*
	 * This function returns the probability for a maximum measurement.
	 */
	private double maxProb(double z) {
		if ((parameters.getLaserRange()) - z <= parameters.getzSmall())
			return (1 / parameters.getzSmall());
		return 0.0;
	}

	/*
	 * This function returns a random value from zero to one.
	 */
	private double randomMeasurementProbability() {
		Random rand = new Random();
		return rand.nextDouble();
	}

	/*
	 * This function returns a random measurement from zero to the number of
	 * pixels that represent the max range of the laser.
	 */
	private int randomMeasurement() {

		Random rand = new Random();
		int value = rand
				.nextInt((int) (parameters.getLaserRange() / (parameters
						.getCmPerPixel() * 0.01)));
		return value;
	}

	/*
	 * This function returns a matrix of double with the probability of the
	 * corresponding measurement.
	 */
	public double[] probabilityDensityFunction(double zExp) {

		double[] prob = new double[(int) (parameters.getLaserRange() / (parameters
				.getCmPerPixel() * 0.01)) + 1];
		double z, h = 1;
		double a1, a2, a3, a4;

		a1 = 2.5;
		a2 = 1;
		a3 = 0.2;
		a4 = 0.05;

		for (int i = 0; i < prob.length; i++) {
			z = 0.05 * i;
			if (zExp >= 0)
				prob[i] = probability(h, a1, a2, a3, a4, z, zExp);
			else
				prob[i] = probabilityWithNoObservation(h, a3, a4, z);
		}

		double area = 0.0; // area is used for the calculation of
							// the integral of the density function

		for (int i = 0; i < prob.length - 1; i++) {
			area = area + (prob[i] + prob[i + 1]) * 0.5;
		}

		while (area > 1.000000001 || area < 0.0000000001) {
			h = Math.pow(area, -1);

			for (int i = 0; i < prob.length; i++) {
				z = 0.01 * parameters.getCmPerPixel() * i;
				prob[i] = h * prob[i];
			}
			area = 0.0;
			for (int i = 0; i < prob.length - 1; i++) {
				area = area + (prob[i] + prob[i + 1]) * 0.5;
			}
		}
		return prob;
	}

	private int mapAngleToMatrixPosition(int angle) {

		int i = 0;

		if (angle == 0) {
			i = 2 * (parameters.getAngleRange() / parameters.getAngleStep());
			i = i / 2;
		} else {
			i = (parameters.getAngleRange() / parameters.getAngleStep())
					+ angle / parameters.getAngleStep();
		}
		System.out.println("Angle " + angle + " i " + i);
		return i;
	}

	private int mapMatrixPositionToAngle(int i) {

		int angle = 0;
		if (i == (parameters.getAngleRange() / parameters.getAngleStep())) {
			angle = 0;
		} else {
			angle = i * parameters.getAngleStep() - parameters.getAngleRange();
		}
		return angle;
	}
	
	public double[] getBruteForceRealMeasurement(){
		for(int i=0;i<55;i++){
			realMeasurements[i] = getObstacleDistance(i*5-135);
		}
		return realMeasurements;
	}
	
	private double getObstacleDistance(int angle){
		double stepsize = 0.01;
		int completeAngle = position.getTheta()+angle;
		double realXChange = stepsize*FastMath.cos(FastMath.toRadians(completeAngle));
		double realYChange = stepsize*FastMath.sin(FastMath.toRadians(-completeAngle));
		for(double i = 0; i<=(10/stepsize); i ++){
			if (map.getRGB(findPixelX(position.getX(), realXChange*i), findPixelY(position.getY(), realYChange*i)) == Color.BLACK.getRGB()){
				picture.setRGB(findPixelX(position.getX(), realXChange*i), findPixelY(position.getY(), realYChange*i), Color.RED.getRGB());
				return i*stepsize;
			}
		}
		return 10.0;
	}
	
	/*
	 * These two functions determine the new pixel coordinates of an endpoint
	 * measurement of a given distance in a given direction (determined by angle).
	 * It uses common trigonometric relations.
	 */
	private int findPixelX(int originX, double realXChange){
		double realOriginX = (originX+0.5)*0.05;
		double realX = realOriginX+realXChange;
		return (int) FastMath.floor(20*realX);
	}
	private int findPixelY(int originY, double realYChange){
		double realOriginY = (originY+0.5)*0.05;
		double realY = realOriginY+realYChange;
		return (int) FastMath.floor(20*realY);
	}
}
