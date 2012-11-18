import java.awt.image.BufferedImage;
import java.util.Random;


public class MainClass {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		//generate a randomly filled measurement array;
		double measurements[] = new double[55];
		Random r = new Random();
		for(int i=0; i<55;i++){
			measurements[i] = r.nextDouble()*10;
		}
		for(double i : measurements){
			System.out.println(i);
		}
		
		//use measurement to create a map of estimated positions
		PositionFinder estimator = new PositionFinder();
		BufferedImage likelihoodmap = estimator.generateMap(measurements);
	}
}
