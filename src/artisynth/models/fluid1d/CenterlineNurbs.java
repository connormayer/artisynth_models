package artisynth.models.fluid1d;

import java.util.ArrayList;
import maspack.geometry.NURBSCurve3d;
import maspack.matrix.*;

public class CenterlineNurbs extends Centerline
{
    int degree = 3;
    //int nPoints;		// super
    int nSegments;
    int nKnots;
    double uMin;
    double uMax;
    double du;
    NURBSCurve3d spline;

    // TODO: my resampling points are not evenly spaced when using the spline, they depend on the original points
    // TODO: length should be an integration along the curve, and not just piecewise, then resample for evenly spaced ds rather than du
    // TODO: also note that this spline is *not* constrained to pass through each control point (though I'd prefer otherwise)
    // how can I design an evenly spaced function s/S = u/U

    public void setVertices(ArrayList<Point3d> pnts)
    {
        points = pnts;
        nPoints = points.size();
        nSegments = nPoints-1;
        nKnots = nPoints+degree-1;

        Vector4d[] wPoints = new Vector4d[nPoints];
        for (int a=0; a<nPoints; a++)
            wPoints[a] = new Vector4d(points.get(a).x, points.get(a).y, points.get(a).z, 1.0);
        spline = new NURBSCurve3d(degree, NURBSCurve3d.OPEN, wPoints, null);
        //spline = new NURBSCurve3d(degree, NURBSCurve3d.OPEN, null, wPoints);
        uMin = spline.findPoint(points.get(0),-1,2);
        uMax = spline.findPoint(points.get(points.size()-1),-1,2);
        du = (uMax-uMin)/((double)nSegments);

        length = calculateLength();		// TODO: this can be done better using a spline method
    }

    public void resample(double ds)
    {
        //double du = ds/length*(uMax-uMin);
        //int nSamples = (uMax-uMin)/du;
        int nSamples = (int)(length/ds);
        resample(nSamples);
    }

    public void resample(int nSegments)
    {
        //Point3d[] pSpline = spline.evalPoints(nSegments);
        // n segments requires n+1 points
        ArrayList<Point3d> pointsNew = new ArrayList<Point3d>(nSegments+1);
        double du = (uMax-uMin)/((double)nSegments);		// du is local for now

        for (int a=0; a<nSegments+1; a++)
        {
            Point3d p = new Point3d();
            spline.eval(p, du*((double)a) + uMin );
            pointsNew.add(p);
        }
        this.setVertices(pointsNew);
    }

    public Vector3d findDirectionAtVertex(int vertexIndex)
    {
        // TODO: this can be better!!
        if ( vertexIndex == 0 )
            return findDirectionBetweenVertices(1,0);
        else if ( vertexIndex == nPoints-1 )
            return findDirectionBetweenVertices(nPoints-2,nPoints-1);
        else
            return findDirectionBetweenVertices(vertexIndex+1,vertexIndex-1);
    }

    public Vector3d findDirectionBetweenVertices(int v1, int v2)
    {
        // TODO: this can be better!!
        // v1: vector head, v2: vector tail
        Vector3d dir = new Vector3d();
        dir.sub(points.get(v1), points.get(v2));
        dir.normalize();					// I think a unit vector is prefered...

        return dir;
    }

    public String getCenterlineName() 
    {
        return String.format("centerline_nurbs_%dpnts", nPoints);
    }

}
