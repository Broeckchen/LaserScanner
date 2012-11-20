import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MainClass {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		BufferedImage map;
		try {
			
			map = ImageIO.read(new File("ressources"+File.separator+"Assignment_5_Grid_Map.png"));
			Raster matrix = map.getData();
			
			/*
			 * laser range is 10 m
			 * every pixel has dimension 5cm x 5cm
			 * laser greatest angle can be 135 degrees(135, -135)
			 * laser angle range is 5
			 * l parameter is 1
			 * b parameter is 2
			 * a small laser measurement is 0.1m 
			 */
			ModelParameters parameters  = new ModelParameters(10, 5, 135, 5, 1, 2, 0.1);
			
			double[] measurements = new double[parameters.getAngleRange()/parameters.getAngleStep()+1]; 
			RobotPosition position = new RobotPosition(150, 200, 45); //define a position for the robot
																	//x = 150 pixel, y =200 pixel and theta = 45 degrees
			MeasurementGenerator generator = new MeasurementGenerator(matrix, position, parameters);
			
			// The computed measurements for the given robot position
			measurements = generator.computeMeasurements();

			for(double i:measurements){
				System.out.print(i+"\t");
			}
			System.out.println();
			
			// use measurement to create a map of estimated positions
			measurements = generator.computeMeasurements();
			
			PositionFinder estimator = new PositionFinder();
			
			//creates a picture of the map created using the gaussian sum method
			double[][] probabilities = estimator.getSummedUpLikelihoodMap();
			BufferedImage likelihoodmap = estimator.generatePicture(probabilities);
			
			File outputFile = new File("summedUpGeneralMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);

			//creates a likelihood map using the given measurement and the gaussian sum map
			probabilities = estimator.generateMap(measurements,false);
			likelihoodmap = estimator.generatePicture(probabilities);
			
			outputFile = new File("summedUpPositionMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);
			
			//creates a picture of the map created using lowest distance method
			probabilities = estimator.getLowestDistanceLikelihoodMap();
			likelihoodmap = estimator.generatePicture(probabilities);
			
			outputFile = new File("lowestDistanceGeneralMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);
		
			//creates a likelihood map using the given measurement and the lowest distance map
			probabilities = estimator.generateMap(measurements,true);
			likelihoodmap = estimator.generatePicture(probabilities);
		
			outputFile = new File("lowestDistancePositionMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);

		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
