package artisynth.models.fluid1d;

import java.util.ArrayList;

import artisynth.models.fluid1d.fluidUtils.FluidUtils;
import maspack.matrix.*;

public class ForceCoupler_1DTo3D_Linear extends ForceCoupler_1DTo3D
{
    double[] value;			// interpolated values

    int[] clSeg;				// location on the centerline
    double[] interpValue;		// interpolation value for that segment value = p_n + interpValue*(p_n+1 - p_n)

    int N;				// number of points


    public void initialize()
    {
        N = targetFaces.size();
        clSeg = new int[N];
        interpValue = new double[N];
        value = new double[N];

        //Centerline cl = fluidSolution.getGeometry().getCenterline();	// make life easier

        // define the interpolation values based on the closest portion of the centerline
        ArrayList<Point3d> clv = fluidSolution.getGeometry().getCenterline().getVertices();	// make life easier
        for (int a=0; a<N; a++)
        {
            Point3d pFace = new Point3d();
            targetFaces.get(a).computeCentroid(pFace);
            Point3d pSeg;
            double dMin=1.0;
            for (int c=0; c<clv.size()-1; c++)
            {
                pSeg = FluidUtils.ClosestPointOnLineSegment(clv.get(c), clv.get(c+1), pFace);
                double d = pFace.distance(pSeg);
                if ( (c==0) || (d<dMin) )
                {
                    dMin = d;
                    clSeg[a] = c;
                    interpValue[a] = clv.get(c).distance(pSeg)/clv.get(c).distance(clv.get(c+1));
                }

            }
        }

        System.out.println("Force Coupler Initialized");
    }

    public void update()
    {
        double p0;
        double p1;

        structureSolution.clearExternalForces();
        for (int n=0; n<N; n++) 
        {
            p0 = fluidSolution.getPressure(clSeg[n]);
            p1 = fluidSolution.getPressure(clSeg[n]+1);
            value[n] = p0 + interpValue[n]*(p1-p0);

            structureSolution.setPressureOnFace( value[n], targetFaces.get(n));
        }
    }
}
