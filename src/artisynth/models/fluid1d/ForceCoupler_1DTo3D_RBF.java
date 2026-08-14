package artisynth.models.fluid1d;

import java.util.ArrayList;

import artisynth.models.fluid1d.fluidUtils.RadialBasisFunction;

import maspack.geometry.*;
import maspack.matrix.*;

public class ForceCoupler_1DTo3D_RBF extends ForceCoupler_1DTo3D
{

    ArrayList<ArrayList<Point3d>> interpolationPoints;
    ArrayList<Point3d> knownPoints;				// a flattened version of interpolationPoints
    double[] knownValues;					// the known values at the interpolation points

    boolean reducePoints = true;

    public void initialize()
    {
        if (reducePoints == true) 
        {
            double dx = fluidSolution.getGeometry().getCenterline().getS(1) - fluidSolution.getGeometry().getCenterline().getS(0);
            ArrayList<ArrayList<Point3d>> newIP = new ArrayList<ArrayList<Point3d>>(); 

            for (ArrayList<Point3d> al : interpolationPoints)
                newIP.add( reduceInterpolationPoints(al, 0.80*dx) );	// note the reduction of dx 

            this.interpolationPoints = newIP;				// the old IP are dropped
        }


        // flatten my interp points
        knownPoints = new ArrayList<Point3d>();
        for (ArrayList<Point3d> points : interpolationPoints)
        {
            knownPoints.addAll(points);
        }
        // allocate for my known values
        knownValues = new double[knownPoints.size()];

        System.out.println("Force Coupler Initialized");

    }

    public ArrayList<Point3d> reduceInterpolationPoints(ArrayList<Point3d> pIn, double dx)
    {
        ArrayList<Point3d> pOut = new ArrayList<Point3d>(pIn.size());
        for (Point3d p : pIn)
            pOut.add(p);

        int i = 0;
        int L = pOut.size();
        while (i<L)		// is this a safe removal loop?
        {
            if (i==0)
            {
                if ( pOut.get(L-1).distance(pOut.get(i+1)) < dx)
                {
                    pOut.remove(i);
                    L--;
                }
                else
                {
                    i++;
                }
            }
            else if (i==L-1)
            {
                if ( pOut.get(i-1).distance(pOut.get(0)) < dx)
                {
                    pOut.remove(i);
                    L--;
                }
                else
                {
                    i++;
                }
            }
            else
            {
                if ( pOut.get(i-1).distance(pOut.get(i+1)) < dx)
                {
                    pOut.remove(i);
                    L--;
                }
                else
                {
                    i++;
                }
            }
        }

        return pOut;
    }

    public void update()
    {
        //SimpleInterpolation();
        rbfInterpolation();
    }

    public void rbfInterpolation()
    {

        // 1) define the points I want to interpolate to --> I can probably speed this up
        ArrayList<Point3d> fsiPoints = new ArrayList<Point3d>(targetFaces.size());			// points
        for (Face face : targetFaces)
        {
            Point3d p = new Point3d();
            face.computeCentroid(p);
            fsiPoints.add(p);
        }

        // 3) define my interpolation values
        int c = 0;
        for (int a=0; a<fluidSolution.getNumberOfPoints(); a++)
        {
            for (int b=0; b<interpolationPoints.get(a).size(); b++)
            {
                knownValues[c] = fluidSolution.getPressure()[a];
                c++;
            }
        }

        // 4) do the actual interpolation
        RadialBasisFunction rbf = new RadialBasisFunction();
        rbf.SetKnownPoints(knownPoints);
        rbf.SetKnownValues(knownValues);
        rbf.SetInterpolationPoints(fsiPoints);
        rbf.Update();

        // 5) apply the interpolation to the faces 
        structureSolution.clearExternalForces();
        for (int a=0; a<targetFaces.size(); a++)
        {
            structureSolution.setPressureOnFace(rbf.GetInterpolationValues()[a], targetFaces.get(a));
        }
    }

    public void simpleInterpolation()
    {
        int nNodes = fluidSolution.getNumberOfPoints();
        structureSolution.clearExternalForces();

        for (Face f : targetFaces)
        {
            int a=0;
            Point3d p = new Point3d();
            f.computeCentroid(p);
            double xF = p.get(0);
            while ( (a < nNodes-1) && (Math.abs(xF - fluidSolution.getGeometry().getPoint(a).x) > Math.abs(xF - fluidSolution.getGeometry().getPoint(a+1).x)) )
            {
                a++;
            }
            structureSolution.setPressureOnFace( fluidSolution.getPressure()[a], f);
        }
    }

    public void setInterpolationPoints(ArrayList<ArrayList<Point3d>> interpPoints)
    {
        interpolationPoints = interpPoints;

    }

    public void setUseReducePoints(boolean useReduced)
    {
        reducePoints = useReduced;
    }

}
