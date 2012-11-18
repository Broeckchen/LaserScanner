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
			
			//use measurement to create a map of estimated positions
			PositionFinder estimator = new PositionFinder();
			BufferedImage likelihoodmap = estimator.generateMap(measurements);
		} catch (IOException e) {

			e.printStackTrace();
		}
		
	}
}
