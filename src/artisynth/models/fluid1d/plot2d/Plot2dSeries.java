package artisynth.models.fluid1d.plot2d;

import java.awt.Color;

public class Plot2dSeries 
{
   public double[] x;
   public double[] y;
   public String name = null;
   
   public boolean drawPoints = true;
   public boolean drawLines = true;
   public Color color = Color.blue;
   public float lineWidth = 2.0f;
   
   public Plot2dSeries()
   {
   }
   
   public Plot2dSeries(String name, double[] x, double[] y)
   {
      this.name = name;
      this.x = x;
      this.y = y;
   }
   
   public double[] getXRange()
   {
      return new double[]{x[0], x[x.length-1]};
   }
   
   public double[] getYRange()
   {
      double yMin = y[0];
      double yMax = y[0];
      for (int i=1; i<y.length; i++)
      {
         if (y[i] < yMin)
            yMin = y[i];
         if (y[i] > yMax)
            yMax = y[i];
      }
      return new double[]{yMin, yMax};
   }
   
   public int getSize()
   {
      return x.length;
   }
}
