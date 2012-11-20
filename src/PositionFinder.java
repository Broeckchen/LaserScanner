import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.util.FastMath;

public class PositionFinder {

	// the assumed standard deviation of the measurement in meter
	private static final double STANDARD_DEVIATION = 0.2; 
	// the distance in meter to an obstacle from where the likelihood will be assumed zero
	private static final double CALCULATION_RANGE = 3* STANDARD_DEVIATION;
	// determines the number of different thetas which will be considered for a single X,Y position
	private static final int THETA_STEPS = 5;
	// the general probability to return the maximum measurement
	private static final double MAX_MEASUREMENT_PROBABILITY = 0.1;
		
	// the calculation range in pixels (on a straight horizontal/vertical line)
	private static final int PIXEL_CALCULATION_RANGE = (int) FastMath.floor(CALCULATION_RANGE / 0.05);
	// the normal distribution of the distance to an obstacle
	private static final NormalDistribution PRIOR = new NormalDistribution(0, STANDARD_DEVIATION);
	// the minimal probability used from the distance distribution
	// equivalent to the probability at the very edge of the calculation range
	// used as the random probability everywhere, where no near obstacle caused a different probability
	private static final double MIN_PROBABILITY = PRIOR.density(CALCULATION_RANGE);
	// the maximal measurement range
	private static final double MAX_MEASUREMENT = 10.0;
	
	// the address of the map
	private static final String MAP_ADDRESS = "ressources"+File.separator+"Assignment_5_Grid_Map.png";
	
	// the original map
	private final BufferedImage map;
	// the two likelihood maps corresponding to the two different approaches
	private double[][] summedUpLikelihoodMap;
	private double[][] lowestDistanceLikelihoodMap;
	// the range of the robot position values
	private static final int THETA = 360;
	private final int X;
	private final int Y;
	
	/*
	 * This function is the constructor.
	 * The initialization of the positionFinder consists of loading the map
	 * and setting the X- and Y-ranges accordingly as well as the initial
	 * creation of the two likelihood maps
	 */
	public PositionFinder() {
		map = loadMap();
		X = map.getWidth();
		Y = map.getHeight();
		summedUpLikelihoodMap = generateMapSummingUp();
		lowestDistanceLikelihoodMap = generateMapLowestDistance();
	}
	
	/*
	 * This function simply loads the map file according to the address in the constant 
	 */
	private BufferedImage loadMap() {
		BufferedImage map = null;
		try {
			map = ImageIO
					.read(new File(MAP_ADDRESS));
		} catch (IOException e) {
			e.printStackTrace();
		}
		return map;
	}

	/*
	 * These functions return either of the two likelihood maps, so they can be plotted.
	 */
	public double[][] getSummedUpLikelihoodMap() {
		return summedUpLikelihoodMap;
	}
	public double[][] getLowestDistanceLikelihoodMap() {
		return lowestDistanceLikelihoodMap;
	}

	/*
	 * This function creates a normalized grid of probabilities
	 * using the method to sum up gaussian probabilities of distances to surrounding obstacles.
	 * To reduce the running time there is a gaussian square created in the beginning according
	 * to the calculation range in pixel, that includes the gaussian probabilities of all pixels
	 * within the calculation range around any black pixel (obstacle).
	 * Thanks to this gaussian square, the gaussian is only calculated a constant number of times;
	 * Every time an black pixel (obstacle) is encountered in the map, the values from the
	 * gaussian square are simply added to all surrounding pixels.
	 * After that, all pixels still with zero probability get the minimal probability assigned as a random factor.
	 * In the very end, the resulting probabilities are normalized in relation to
	 * the maximum probability encountered during the generation of the probability grid.
	 * This results in a grid of probabilities in the interval from 0.0 to 1.0;
	 */
	private double[][] generateMapSummingUp() {
		double[][] gaussianSquare = gaussianSquare();

		double maxLikelihood = 0.0;
		double[][] probabilities = new double[X][Y];
		for (int i = 0; i < X; i++) {
			for (int j = 0; j < Y; j++) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {
					for (int k = i - PIXEL_CALCULATION_RANGE;
							k <= i + PIXEL_CALCULATION_RANGE; k++) {
						for (int l = j - PIXEL_CALCULATION_RANGE;
								l <= j + PIXEL_CALCULATION_RANGE; l++) {
							if (k >= 0 && k < X && l >= 0 && l < Y) {
								probabilities[k][l] += gaussianSquare
										[i - k + PIXEL_CALCULATION_RANGE]
										[j - l + PIXEL_CALCULATION_RANGE];
								if (probabilities[k][l] > maxLikelihood) {
									maxLikelihood = probabilities[k][l];
								}
							}
						}
					}
				}
			}
		}
		for(int i=0;i<X;i++){
			for(int j=0;j<Y;j++){
				if(probabilities[i][j] == 0){
					probabilities[i][j] = MIN_PROBABILITY;
				}
			}
		}

		probabilities = normaliseProbabilities(probabilities, maxLikelihood);

		return probabilities;
	}
		
	/*
	 * This function creates a normalized grid of probabilities
	 * using the method to search for the lowest distance to an obstacle.
	 * To reduce the running time there is a gaussian square created in the beginning according
	 * to the calculation range in pixel, that includes the gaussian probabilities of all pixels
	 * within the calculation range around any black pixel (obstacle).
	 * Thanks to this gaussian square, the gaussian is only calculated a constant number of times;
	 * Every time an black pixel (obstacle) is encountered in the map, the probability values for
	 * all pixels around the grid, are set to the according value from the gaussian square if there
	 * is not already a higher probability set. A higher set probability would have meant, that another
	 * obstacle has been encountered before, which is nearer to that pixel.
	 * After that, all pixels still with zero probability get the minimal probability assigned as a random factor.
	 * In the very end, the resulting probabilities are normalized in relation to
	 * the maximum probability encountered during the generation of the probability grid.
	 * This results in a grid of probabilities in the interval from 0.0 to 1.0;
	 */
	private double[][] generateMapLowestDistance() {
		double maxLikelihood = 0.0;
		double[][] gaussianSquare = gaussianSquare();

		double[][] probabilities = new double[X][Y];
		for (int i = 0; i < X; i++) {
			for (int j = 0; j < Y; j++) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {
					for (int k = i - PIXEL_CALCULATION_RANGE;
							k <= i + PIXEL_CALCULATION_RANGE; k++) {
						for (int l = j - PIXEL_CALCULATION_RANGE;
								l <= j + PIXEL_CALCULATION_RANGE; l++) {
							if (k >= 0 && k < X && l >= 0 && l < Y) {
								double newProbability = gaussianSquare
										[i - k + PIXEL_CALCULATION_RANGE]
										[j - l + PIXEL_CALCULATION_RANGE];
								if(probabilities[k][l] < newProbability){
									probabilities[k][l] = newProbability;
								}
								if (probabilities[k][l] > maxLikelihood) {
									maxLikelihood = probabilities[k][l];
								}
							}
						}
					}
				}
			}
		}
		for(int i=0;i<X;i++){
			for(int j=0;j<Y;j++){
				if(probabilities[i][j] == 0){
					probabilities[i][j] = MIN_PROBABILITY;
				}
			}
		}

		probabilities = normaliseProbabilities(probabilities, maxLikelihood);

		return probabilities;
	}
	
	/*
	 * This function returns a square grid of gaussian probabilities.
	 * The size of the square is dependent on the calculation range,
	 * e.g. it is chosen in such a way, that the distance from the middle point of the square
	 * to the middle point of either the first or last row or column is exactly the pixel calculation range.
	 * For each position the grid contains the gaussian density of the distance to the middle of the square.
	 * If the distance is bigger than the calculation range, the value at the position is set to 0.0.
	 * This way, all relevant values of the gaussian distribution are contained in the resulting square.
	 */
	public double[][] gaussianSquare() {
		int gaussianSquareSize = 2 * PIXEL_CALCULATION_RANGE + 1;

		double[][] gaussianSquare = new double[gaussianSquareSize][gaussianSquareSize];
		for (int i = 0; i < gaussianSquareSize; i++) {
			for (int j = 0; j < gaussianSquareSize; j++) {
				double distance = distance(PIXEL_CALCULATION_RANGE,
						PIXEL_CALCULATION_RANGE, i, j);
				if (distance <= CALCULATION_RANGE) {
					gaussianSquare[i][j] = PRIOR.density(distance);
				}
			}
		}
		return gaussianSquare;
	}
	
	/*
	 * This function normalizes a grid of probabilities of the size of the map
	 * according to the given maximum likelihood in that grid. By dividing every value by the 
	 * maximum likelihood, the resulting probabilities all lie in the range of 0.0 to 1.0.
	 */
	private double[][] normaliseProbabilities(double[][] probabilities, double maxLikelihood){
		for (int i = 0; i < X; i++) {
			for (int j = 0; j < Y; j++) {
				if(probabilities[i][j]==0){
					probabilities[i][j] = MIN_PROBABILITY;
				}
				probabilities[i][j] = probabilities[i][j]/maxLikelihood;
			}
		}
		return probabilities;
	}
	
	/*
	 * This function generates a probability grid of possible robot poses.
	 * For every position in the grid, it denotes the highest probability of
	 * all possible robot orientations at that position.
	 * It first determines, which map to use according to the given boolean.
	 * The maps have been generated at the initialization of the position finder and can be used
	 * again and again for any measurement to come.
	 * Then for each position of the map, the highest likelihood of all different orientations
	 * is determined with the following method:
	 * For each value of measurement array, the according endpoint pixel is determined and the according
	 * probability is weighted and added to the accumulated likelihood. If the endpoint lies outside the map,
	 * the minimal probability is used instead. If the measurement is the max measurement, an additional bonus
	 * is added since the max measurement very likely in general.
	 * In the end, the probabilities are normalized again to use to the range of 0.0 to 1.0,
	 * mainly to have the resulting picture utilize as much as possible of the grey range.
	 */
	public double[][] generateMap(double[] measurements, boolean lowestDistanceApproach) {
		double[][] probabilities;
		if (lowestDistanceApproach) {
			probabilities = lowestDistanceLikelihoodMap;
		} else {
			probabilities = summedUpLikelihoodMap;
		}

		double[][] likelihoods = new double[X][Y];
		double maxLikelihood=0.0;
		
		for(int i=0;i<X;i++){
			System.out.println(i);
			for(int j=0;j<Y;j++){
				if(map.getRGB(i, j) == Color.WHITE.getRGB()){
					for(int k=0;k<THETA;k=k+THETA_STEPS){
						double likelihood = 0.0;
						for(int l=0;l<measurements.length;l++){
							int angle = k-135+l*5;
							double distance = measurements[l];
							int endpointX = findPixelX(i,angle,distance);
							int endpointY = findPixelY(j,angle,distance);
							if(endpointX<0 || endpointX>=X || endpointY<0 || endpointY>=Y || probabilities[endpointX][endpointY]==0){
								likelihood += MIN_PROBABILITY;
							}else{
								likelihood += probabilities[endpointX][endpointY];
							}
							if(distance == MAX_MEASUREMENT){
								likelihood += MAX_MEASUREMENT_PROBABILITY;
							}
						}
						if(likelihood>likelihoods[i][j]){
							likelihoods[i][j] = likelihood;
						}
					}
					if(likelihoods[i][j]>maxLikelihood){
						maxLikelihood = likelihoods[i][j];
					}
				}
			}
		}
		normaliseProbabilities(likelihoods, maxLikelihood);
		
		return likelihoods;
	}
	
	/*
	 * This function generates a greyscale picture of a probability grid.
	 * It assumes the probability grid to have the ranges of the original map
	 * and all probabilities to lie between 0.0 and 1.0;
	 */
	public BufferedImage generatePicture(double[][] probabilities){
		BufferedImage likelihoodMap = new BufferedImage(X, Y,
				BufferedImage.TYPE_BYTE_GRAY);
		for (int i = 0; i < X; i++) {
			for (int j = 0; j < Y; j++) {
				float rgbFloat = 1 - (float) probabilities[i][j];
				Color color = new Color(rgbFloat, rgbFloat, rgbFloat);
				likelihoodMap.setRGB(i, j, color.getRGB());
			}
		}
		return likelihoodMap;
	}

	/*
	 * These two functions determine the new pixel coordinates of an endpoint
	 * measurement of a given distance in a given direction (determined by angle).
	 * It uses common trigonometric relations.
	 */
	private int findPixelX(int originX, int angle, double distance){
		double realOriginX = (originX+0.5)*0.05;
		double radAngle = FastMath.toRadians(angle);
		double realX = realOriginX+distance*FastMath.cos(radAngle);
		return (int) FastMath.floor(20*realX);
	}
	private int findPixelY(int originY, int angle, double distance){
		double realOriginY = (originY+0.5)*0.05;
		double radAngle = FastMath.toRadians(angle);
		double realY = realOriginY+distance*FastMath.sin(radAngle);
		return (int) FastMath.floor(20*realY);
	}
	
	/*
	 * This function computes the distance of two given pixels in meter
	 * according to the assumption that 1 pixel is 0.05m*0.05m
	 */
	private double distance(int x1, int y1, int x2, int y2) {
		double real_x1 = (x1 + 0.5) * 0.05;
		double real_x2 = (x2 + 0.5) * 0.05;
		double real_y1 = (y1 + 0.5) * 0.05;
		double real_y2 = (y2 + 0.5) * 0.05;
		return FastMath.sqrt(FastMath.pow(real_x2 - real_x1, 2)
				+ FastMath.pow(real_y2 - real_y1, 2));
	}
}