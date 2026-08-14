package artisynth.models.fluid1d.plot2d;

import java.util.ArrayList;

import artisynth.core.modelbase.MonitorBase;
import artisynth.models.fluid1d.FluidSolution_1D;
import artisynth.models.fluid1d.plot2d.Subplot.PlotType;

public class PlotMonitor extends MonitorBase
{

   Plot2d plot = new Plot2d();
   ArrayList<Subplot> subplots = new ArrayList<Subplot>();
   FluidSolution_1D fs;
   
   public void setFluidSolution(FluidSolution_1D fluidSolution)
   {
      fs = fluidSolution;
   }
   
   Plot2dSeries createSeries(String name, double[] x, double[] y)
   {
      // safely....
      if (x==null)
      {
         x = new double[]{0.0, 1.0};
         y = new double[]{0.0, 1.0};
      }
      else if (y==null)
      {
         y = new double[x.length];
      }
      Plot2dSeries series = new Plot2dSeries(name, x, y);
      return series;
   }
   
   public Plot2dAxes addSubplot_Area(String name)
   {
      Plot2dAxes ax = new Plot2dAxes();
      ax.setName (name);
      plot.addAxis (ax);
      
      ax.addSeries( createSeries("area", fs.getGeometry().getCenterline().getS(), fs.getGeometry().getArea() ) );
      ax.addSeries( createSeries("area", fs.getGeometry().getCenterline().getS(), fs.getField("areaReal") ) );
      ax.updateRangeFromData();
      
      Subplot subplot = new Subplot();
      subplot.plotType = PlotType.area;
      subplot.name = name;
      subplot.axes = ax;
      subplots.add (subplot);
      
      return ax;
      
   }
   
   public Plot2dAxes addSubplot(String name, String[] terms)
   {
      Plot2dAxes ax = new Plot2dAxes();
      ax.setName (name);
      plot.addAxis (ax);
      
      for (int i=0; i<terms.length; i++)
      {
         Plot2dSeries s = createSeries(terms[i], fs.getGeometry().getCenterline().getS(), fs.getField(terms[i]));
         ax.addSeries(s);
      }
      ax.updateRangeFromData();
      
      Subplot subplot = new Subplot();
      subplot.plotType = PlotType.field;
      subplot.name = name;
      subplot.axes = ax;
      subplots.add (subplot);
      
      return ax;
   }
   
   public void initialize()
   {
      plot.buildPlot ();
   }
   
   public void apply (double t0, double t1) 
   {
      for (Subplot subplot : subplots)
      {
         Plot2dAxes ax = subplot.axes;
         
         if (subplot.plotType == PlotType.area)
         {
            ax.series.get(0).x = fs.getGeometry().getCenterline().getS();
            ax.series.get(0).y = fs.getGeometry().getArea();
            ax.series.get(1).x = fs.getGeometry().getCenterline().getS();
            ax.series.get(1).y = fs.getField("areaReal");
         }
         else if (subplot.plotType == PlotType.field)
         {   
            for (Plot2dSeries s : ax.series)
            {
               // this is a total kludge!
               s.x = fs.getGeometry().getCenterline().getS();
               s.y = fs.getField(s.name);
            }
            ax.updateRangeFromData();  
         }
      }
//      for (Plot2dAxes ax : plot.axes)
//      {
//         for (Plot2dSeries s : ax.series)
//         {
//            // this is a total kludge!
//            s.x = fs.getGeometry().getCenterline().getS();
//            s.y = fs.getField(s.name);
//         }
//         ax.updateRangeFromData();
//      }
      plot.update();
   }

}

class Subplot
{
   public enum PlotType {area, perimeter, field};
   PlotType plotType;
   String[] terms;
   String name;
   Plot2dAxes axes;
}
