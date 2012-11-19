import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.apache.commons.math3.distribution.NormalDistribution;
import org.apache.commons.math3.util.FastMath;

public class PositionFinder {

	//the assumed standard deviation of the measurement in meter
	private static final double STANDARD_DEVIATION = 0.25; 
	// the distance in meter to an obstacle from where the likelihood will be assumed zero
	private static final double CALCULATION_RANGE = 4* STANDARD_DEVIATION; 
	private static final int PIXEL_CALCULATION_RANGE = (int) FastMath.floor(CALCULATION_RANGE / 0.05);
	private static final NormalDistribution PRIOR = new NormalDistribution(0, STANDARD_DEVIATION);
	private static final double MIN_PROBABILITY = PRIOR.density(CALCULATION_RANGE);
	private static final double MAX_MEASUREMENT = 10.0;
	private static final double MAX_MEASUREMENT_PROBABILITY = 0.1;
	
	private static final int X = 680;
	private static final int Y = 400;
	private static final int THETA = 360;
	private static final int THETA_STEPS = 1;
	
	private BufferedImage map;
	private double maxLikelihood;
	private double maxMeasurementProbability;
	
	public PositionFinder() {
		map = loadMap();
	}

	public double[][] generateMapSummingUp() {
		double[][] gaussianSquare = gaussianSquare();

		maxLikelihood = 0.0;
		double[][] probabilities = new double[X][Y];
		for (int i = 0; i < X; i++) {
			System.out.println(i);
			for (int j = 0; j < Y; j++) {
				if (map.getRGB(i, j) == Color.BLACK.getRGB()) {
					for (int k = i - PIXEL_CALCULATION_RANGE; k <= i
							+ PIXEL_CALCULATION_RANGE; k++) {
						for (int l = j - PIXEL_CALCULATION_RANGE; l <= j
								+ PIXEL_CALCULATION_RANGE; l++) {
							if (k >= 0 && k < X && l >= 0 && l < Y) {
								probabilities[k][l] += gaussianSquare[i - k
										+ PIXEL_CALCULATION_RANGE][j - l
										+ PIXEL_CALCULATION_RANGE];
								if (probabilities[k][l] > maxLikelihood) {
									maxLikelihood = probabilities[k][l];
								}
							}
						}
					}
				}
			}
		}

		probabilities = normaliseProbabilities(probabilities);

		return probabilities;
	}

	public double[][] generateMapLowestDistance() {
		maxLikelihood = 0.0;
		double[][] probabilities = new double[X][Y];
		for (int i = 0; i < X; i++) {
			System.out.println(i);
			for (int j = 0; j < Y; j++) {
				double smallestDistance = 3 * CALCULATION_RANGE;
				for (int k = i - PIXEL_CALCULATION_RANGE; k <= i
						+ PIXEL_CALCULATION_RANGE; k++) {
					for (int l = j - PIXEL_CALCULATION_RANGE; l <= j
							+ PIXEL_CALCULATION_RANGE; l++) {
						if (k >= 0 && k < X && l >= 0 && l < Y) {
							if (map.getRGB(k, l) == Color.BLACK.getRGB()) {
								double newDistance = distance(k, l, i, j);
								if (newDistance < smallestDistance) {
									smallestDistance = newDistance;
								}
							}
						}
					}
				}
				probabilities[i][j] = PRIOR.density(smallestDistance);
				if (probabilities[i][j] > maxLikelihood) {
					maxLikelihood = probabilities[i][j];
				}
			}
		}

		probabilities = normaliseProbabilities(probabilities);

		return probabilities;
	}
	
	private double[][] normaliseProbabilities(double[][] probabilities){
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
	
	private double[][] gaussianSquare() {
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

	public double[][] generateMap(double[] measurements, boolean mode) {
		double[][] probabilities;
		if (mode) {
			probabilities = generateMapLowestDistance();
		} else {
			probabilities = generateMapSummingUp();
		}

		double[][] likelihoods = new double[X][Y];
		maxMeasurementProbability = MAX_MEASUREMENT_PROBABILITY/maxLikelihood;
		
		maxLikelihood=0.0;	
		
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
							if(endpointX<0 || endpointX>=X || endpointY<0 || endpointY>=Y){
								likelihood += MIN_PROBABILITY;
							}else{
								likelihood += probabilities[endpointX][endpointY];
							}
							if(distance == MAX_MEASUREMENT){
								likelihood += maxMeasurementProbability;
							}
						}
						if(likelihood>likelihoods[i][j]){
							likelihoods[i][j] = likelihood;
						}
					}
					if(likelihoods[i][j]>maxLikelihood){
						System.out.println("new highest likelihood found:" + likelihoods[i][j]);
						maxLikelihood = likelihoods[i][j];
					}
				}
			}
		}
		normaliseProbabilities(likelihoods);
		
		return likelihoods;
	}
	
	public BufferedImage generatePicture(double[][] probabilities){
		BufferedImage likelihoodMap = new BufferedImage(680, 400,
				BufferedImage.TYPE_BYTE_GRAY);
		for (int i = 0; i < X; i++) {
			for (int j = 0; j < Y; j++) {
				float rgbFloat = (float) probabilities[i][j];
				Color color = new Color(rgbFloat, rgbFloat, rgbFloat);
				likelihoodMap.setRGB(i, j, color.getRGB());
			}
		}
		return likelihoodMap;
	}

	private int findPixelX(int originX, int angle, double distance){
		double realOriginX = (originX+0.5)*0.05;
		double realX = realOriginX+distance*FastMath.cos(angle);
		return (int) FastMath.floor(20*realX);
	}
	
	private int findPixelY(int originY, int angle, double distance){
		double realOriginY = (originY+0.5)*0.05;
		double realY = realOriginY+distance*FastMath.sin(angle);
		return (int) FastMath.floor(20*realY);
	}
	
	/**
	 * Computes the distance of two given pixels
	 * 
	 * @param x1
	 *            the x-position of the first pixel
	 * @param y1
	 *            the y-position of the first pixel
	 * @param x2
	 *            the x-position of the second pixel
	 * @param y2
	 *            the y-position of the second pixel
	 * @return the distance of the midpoints of the pixels in meter according to
	 *         the relation 1 pixel = 0.05m*0.05m
	 */
	private double distance(int x1, int y1, int x2, int y2) {
		double real_x1 = (x1 + 0.5) * 0.05;
		double real_x2 = (x2 + 0.5) * 0.05;
		double real_y1 = (y1 + 0.5) * 0.05;
		double real_y2 = (y2 + 0.5) * 0.05;
		return FastMath.sqrt(FastMath.pow(real_x2 - real_x1, 2)
				+ FastMath.pow(real_y2 - real_y1, 2));
	}

	private BufferedImage loadMap() {
		BufferedImage map = null;
		try {
			map = ImageIO
					.read(new File("ressources/Assignment_5_Grid_Map.png"));
		} catch (IOException e) {
			System.out.println(e.getMessage());
		}
		return map;
	}
}