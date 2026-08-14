package artisynth.models.fluid1d;

import java.util.ArrayList;

import artisynth.models.fluid1d.fileIO.CSV;
import artisynth.models.fluid1d.fluidUtils.FluidUtils;
import maspack.matrix.*;

//TODO: make this an extension of polyline

public abstract class Centerline 
{
    // this defines s = s(x,y,z)
    ArrayList<Point3d> points;
    double length;
    double[] distances;			// this is the distance along the curve
    int nPoints;

    // -- abstract methods -- //
    public abstract void resample(double ds);
    public abstract void resample(int nSegments);
    public abstract Vector3d findDirectionAtVertex(int vertexIndex);
    public abstract Vector3d findDirectionBetweenVertices(int v1, int v2);
    public abstract String getCenterlineName();
    //public abstract Centerline getCenterlineSection(int iStart, int iEnd);
    // probably calculate length can be done with more accuracy in the centerline spline

    public Centerline()
    {
    }

    public void setVertices(ArrayList<Point3d> pnts)
    {
        points = pnts;
        nPoints = points.size();
        length = calculateLength();
    }

    public ArrayList<Point3d> getVertices()
    {
        return points;
    }

    public double calculateLength()
    {
        distances = new double[points.size()];
        distances[0] = 0.0;

        for (int a=1; a<points.size(); a++)
        {
            distances[a] = distances[a-1] + points.get(a-1).distance(points.get(a));
            //l = l + points.get(a).distance(points.get(a+1));
        }

        return distances[points.size()-1];
    }

    public double getLength()
    {
        return distances[points.size()-1];
    }

    public double getS(int index)
    {
        return distances[index];
    }

    public double[] getS()
    {
        return distances;
    }

    public Point3d findClosestPoint(Point3d p)
    {
        double s = findClosestS(p);
        return getWorldLocation(s);
    }

    public double findClosestS(Point3d p)
    {
        // p is some world point, not necessarily on the centerline. This finds the closest centerline point, and then the best s-value
        // this and the above function assume the centerline is linear!  I really should remove my nurbs centerline...
        Point3d pSeg;
        double sMin = 0.0;
        double dMin = 1.0;
        for (int c=0; c<points.size()-1; c++)
        {
            pSeg = FluidUtils.ClosestPointOnLineSegment(points.get(c), points.get(c+1), p);
            double d = p.distance(pSeg);
            if ( (c==0) || (d<dMin) )
            {
                dMin = d;
                sMin = getS(c) + points.get(c).distance(pSeg);
            }
        }
        return sMin;
    }

    public Point3d getWorldLocation(double s)
    {
        Point3d pnt = new Point3d();

        if ((s < 0.0) || (s > length))
        {
            return null;
        }
        else if (s == 0.0)
        {
            return points.get(0);
        }

        // find the two points that I need to interpolate between
        int a=0;
        while (s > distances[a])
            a++;

        // interpolate between a-1 and a: (lp - la-1)/(la - la-1) = c = (xp - xa-1)/(xa - xa-1)
        double c = (s-distances[a-1])/(distances[a]-distances[a-1]);
        pnt.x = points.get(a).x * c + points.get(a-1).x * (1.0 - c);
        pnt.y = points.get(a).y * c + points.get(a-1).y * (1.0 - c);
        pnt.z = points.get(a).z * c + points.get(a-1).z * (1.0 - c);

        return pnt;
    }

    public Centerline getCenterlineSection(int iStart, int iEnd) 
    {
        ArrayList<Point3d> ps = new ArrayList<Point3d>(iEnd-iStart);
        for (int i=iStart; i<=iEnd; i++)
            ps.add(points.get(i));
        Centerline cl;		// = this.getClass().newInstance();
        if (this instanceof CenterlineNurbs)
            cl = new CenterlineNurbs();
        else
            cl = new CenterlineLinear();
        cl.setVertices(ps);

        return cl;
    }

    public Centerline getCenterlineSection(double sStart, double sEnd) 
    {
        // find indices between which sStart occurs
        // find indices between which sEnd occurs
        // find the world points (
        int iStart = 0;	// mark the indices to insert the points *before*
        int iEnd = 0;
        for (int i=0; i<nPoints; i++)
        {
            if (sStart>distances[i])
                iStart++;
            if (sEnd>distances[i])
                iEnd++;
        }

        ArrayList<Point3d> ps = new ArrayList<Point3d>(iEnd-iStart);
        if (iStart>0)
        {
            Point3d pStart = getWorldLocation(sStart);
            ps.add(pStart);
        }
        for (int i=iStart; i<=iEnd-1; i++)
            ps.add(points.get(i));
        if (iEnd<nPoints)
        {
            Point3d pEnd = getWorldLocation(sEnd);
            ps.add(pEnd);
        }

        Centerline cl;		// = this.getClass().newInstance();
        if (this instanceof CenterlineNurbs)
            cl = new CenterlineNurbs();
        else
            cl = new CenterlineLinear();
        cl.setVertices(ps);

        return cl;
    }

    public int getNumberOfPoints()
    {
        return nPoints;
    }

    public void reverseDirection()
    {
        ArrayList<Point3d> pointsRev = new ArrayList<Point3d>(points.size());
        for (int i=0; i<nPoints; i++)
        {
            pointsRev.add(points.get(nPoints-1-i));
        }
        points = pointsRev;
    }

    public void readVertices(String filename)
    {
        CSV csv = new CSV();
        csv.Read(filename);
        ArrayList<Point3d> points = new ArrayList<Point3d>(csv.nRows);
        for (int a=0; a<csv.nRows; a++)
        {
            points.add(new Point3d(csv.data[a]));
        }

        setVertices(points);
    }

    public void writeVertices(String filename)
    {
        double[][] data = artisynth.models.fluid1d.fluidUtils.FluidUtils.Point3dToDoubleArray(points);
        String[] headers = {"x", "y", "z"};
        CSV.Write(filename,headers, data);
    }

}
