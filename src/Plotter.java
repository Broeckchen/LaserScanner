import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.annotations.XYTextAnnotation;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;

import com.lowagie.text.Font;


public class Plotter {
	
	protected String title;
	protected String axisX;
	protected String axisY;
	protected XYSeries data;
	
	public Plotter(String title, String axisX, String axisY, XYSeries data){
		this.title = title;
		this.axisX = axisX;
		this.axisY = axisY;
		this.data = data;
	}
	
	public String getTitle() {
		return title;
	}

	public void setTitle(String title) {
		this.title = title;
	}

	public String getAxisX() {
		return axisX;
	}

	public void setAxisX(String axisX) {
		this.axisX = axisX;
	}

	public String getAxisY() {
		return axisY;
	}

	public void setAxisY(String axisY) {
		this.axisY = axisY;
	}

	public XYSeries getData() {
		return data;
	}

	public void setData(XYSeries data) {
		this.data = data;
	}

	public void printPlot(){
		XYDataset dataSet = new XYSeriesCollection(data);
		JFreeChart chart = plotVector(dataSet);
        ChartPanel panel = new ChartPanel(chart, true, true, true, false, true);
        panel.setPreferredSize(new java.awt.Dimension(800, 500));
        ApplicationFrame  frame = new ApplicationFrame(title);
        frame.setContentPane(panel);
        frame.pack();
        RefineryUtilities.centerFrameOnScreen(frame);
        frame.setVisible(true);
	}
	
	public JFreeChart plotVector(XYDataset dataSet){
		
		final XYItemRenderer renderer1 = new StandardXYItemRenderer();
        final NumberAxis rangeAxis1 = new NumberAxis(axisX);
        final XYPlot subplot1 = new XYPlot(dataSet, null, rangeAxis1, renderer1);
        subplot1.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
        java.awt.Font font =new java.awt.Font("SansSerif", Font.NORMAL, 9);
        final XYTextAnnotation annotation = new XYTextAnnotation("Hello!", 50.0, 10000.0);
        annotation.setFont(font);
        annotation.setRotationAngle(Math.PI / 4.0);
        subplot1.addAnnotation(annotation);
      
        // parent plot...
        final CombinedDomainXYPlot plot = new CombinedDomainXYPlot(new NumberAxis(axisY));
        plot.setGap(10.0);
        
        // add the subplots...
        plot.add(subplot1, 1);
        plot.setOrientation(PlotOrientation.VERTICAL);

        // return a new chart containing the overlaid plot...
        return new JFreeChart("Probability Density Function",
                              JFreeChart.DEFAULT_TITLE_FONT, plot, true);

	}
}
