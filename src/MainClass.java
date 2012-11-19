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
			
			ModelParameters parameters  = new ModelParameters(10, 5, 135, 5, 1, 2, 0.1 );
			
			double[] measurements = new double[parameters.getAngleRange()/parameters.getAngleStep()+1]; 
			RobotPosition position = new RobotPosition(10, 10, 0); //define a position for the robot
			MeasurementGenerator generator = new MeasurementGenerator(matrix, position, parameters);
			
			// The computed measurements for the given robot position
			measurements = generator.computeMeasurements();
			
			for(double i : measurements){
				System.out.println(i);
			}
			
			// use measurement to create a map of estimated positions
			PositionFinder estimator = new PositionFinder();
			
			double[][] probabilities = estimator.getSummedUpLikelihoodMap();
			BufferedImage likelihoodmap = estimator.generatePicture(probabilities);
			
			File outputFile = new File("summedUpGeneralMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);
		
			probabilities = estimator.generateMap(measurements,false);
			likelihoodmap = estimator.generatePicture(probabilities);
		
			outputFile = new File("summedUpPositionMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);
			
			probabilities = estimator.getLowestDistanceLikelihoodMap();
			likelihoodmap = estimator.generatePicture(probabilities);
			
			outputFile = new File("lowestDistanceGeneralMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);
		
			probabilities = estimator.generateMap(measurements,true);
			likelihoodmap = estimator.generatePicture(probabilities);
		
			outputFile = new File("lowestDistancePositionMap.png");
			ImageIO.write(likelihoodmap, "png", outputFile);

		} catch (IOException e) {

			e.printStackTrace();
		}
	}
}
