package artisynth.models.fluid1d;

import maspack.matrix.*;
import java.io.*;

public class Geometry_1D
{
    // Area = Area(s)
    // s = s(x,y,z) --> s has a length, and for any given point along that length one can get the 3d-coords

    Centerline centerline;		//
    double[] area;			// should contain an area value for each point in centerline
    double[] perim;			// the wetted perimeter

    public double percentClosed()
    {
        double pc = 0.0;					// percent closed
        double pcMin = 0.0;

        for (int a=0; a<area.length; a++)
        {
            pc = ((area[0]-area[a])/area[0])*100.0;
            if ( (pc > pcMin) || (a==0) )
                pcMin = pc;
        }
        return pcMin;
    }

    public void setCenterline(Centerline cl)
    {
        centerline = cl;
        area = new double[centerline.getNumberOfPoints()];
        perim = new double[centerline.getNumberOfPoints()];
    }

    public Centerline getCenterline()
    {
        return centerline;
    }

    public void setArea(double[] areaValues)
    {
        area = areaValues;
    }

    public double[] getArea()
    {
        return area;
    }

    public double getArea(int index)
    {
        return area[index];
    }

    public void setArea(int index, double area)
    {
        this.area[index] = area;
    }

    public Point3d getPoint(int index)
    {
        return centerline.getVertices().get(index);
    }

    public double getPerimeter(int i)
    {
        return perim[i];
    }

    public double[] getPerimeter()
    {
        return perim;
    }

    public void setPerimeter(int index, double perimeter)
    {
        this.perim[index] = perimeter;
    }

    public void setPerimeter(double[] perimeter)
    {
        this.perim = perimeter;
    }

    public int getNumberOfPoints()
    {
        return centerline.getNumberOfPoints();
    }

    public void writeGeometry(String filename)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));

            file.print("x,y,z,s,area,perimeter\n");
            for (int a=0; a<area.length; a++)
            {
                file.printf("%f,%f,%f,%f,%f,%f\n", centerline.getVertices().get(a).x, centerline.getVertices().get(a).y, centerline.getVertices().get(a).z, centerline.getS(a), area[a], perim[a]);
            }

            file.close();
        }
        catch(Exception e)
        {
        }
    }

    public Geometry_1D deepCopy()
    {
        Geometry_1D geom = new Geometry_1D();
        geom.setCenterline(this.centerline);		// TODO: warning! Not a proper deep copy!
        geom.setArea(artisynth.models.fluid1d.fluidUtils.FluidUtils.deepCopy_doubleArray(this.area));
        geom.setPerimeter(artisynth.models.fluid1d.fluidUtils.FluidUtils.deepCopy_doubleArray(this.perim));

        return geom;
    }

    public Geometry_1D getGeometrySection(int iStart, int iEnd)
    {
        int N = iEnd-iStart+1;
        double[] area = new double[N];
        double[] peri = new double[N];
        for (int i=0; i<N; i++)
        {
            area[i] = this.area[iStart+i];
            peri[i] = this.perim[iStart+i];
        }

        Geometry_1D geom = new Geometry_1D();
        geom.setCenterline(this.centerline.getCenterlineSection(iStart, iEnd));
        geom.setArea(area);
        geom.setPerimeter(peri);

        return geom;
    }


}
