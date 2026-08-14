package artisynth.models.fluid1d.plot2d;

import java.awt.Color;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.RootModel;

public class Plot2dDemo extends RootModel 
{
   int N = 50;
   double[] x = new double[N];
   double[] y1 = new double[N];
   double[] y2 = new double[N];
   double[] y3 = new double[N];
   double[] y4 = new double[N];
   double[][] vals = new double[N][2];
   double xMin = 0.0;
   double xMax = 2.0;
   double dx = (xMax-xMin)/((double)(N-1));
   
   Plot2d plot2d = new Plot2d();
   Plot2dAxes pax1 = new Plot2dAxes();
   Plot2dAxes pax2 = new Plot2dAxes();
   Plot2dAxes pax3 = new Plot2dAxes();
   
   public Plot2dDemo(String name)
   {
      super(name);
      
      MechModel mechModel = new MechModel();
      this.add(mechModel);
      
      for (int n=0; n<N; n++)
         x[n] = xMin + dx*n;
      func(0.0);
      
      Plot2dSeries series1 = new Plot2dSeries("sin func", x, y1);
      Plot2dSeries series2 = new Plot2dSeries("sin func reversed", x, y2);
      series2.color = Color.green;
      series2.drawPoints = false;
      Plot2dSeries series3 = new Plot2dSeries("sin func reversed", x, y3);
      Plot2dSeries series4 = new Plot2dSeries("delta func", x, y4);
      pax1.series.add (series1);
      pax1.series.add (series2);
      pax1.updateRangeFromData();
      
      pax2.series.add (series3);
      pax2.updateRangeFromData();
      pax2.setYRange (-1.5, 1.0);
      
      pax3.series.add (series4);
      pax3.setXRange (xMin, xMax);
      pax3.setYRange (-1.5, 1.5);
      
      plot2d.addAxis (pax1);
      plot2d.addAxis (pax2);
      plot2d.addAxis (pax3);
      plot2d.buildPlot ();
      
   }
   
   public void func(double time)
   {
      for (int n=0; n<N; n++)
      {
         y1[n] = Math.sin(2.0*Math.PI*(x[n]+time));
         y2[n] = Math.sin(2.0*Math.PI*(x[n]-time));
         y3[n] = Math.sin(2.0*Math.PI*x[n])*Math.sin(2.0*Math.PI*time);
         if (Math.abs(time-x[n]) % (xMax-xMin) < 0.1)
            y4[n] = 1.0;
         else
            y4[n] = 0.0;
      }
   }
   
   
   @Override
   public StepAdjustment advance( double t0, double t1, int flags) 
   {
      func(t1);
      plot2d.update();
      //plotFrame.repaint ();
      return super.advance(t0, t1, flags);
   }
   
   
}
