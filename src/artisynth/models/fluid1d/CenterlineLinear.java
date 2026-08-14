package artisynth.models.fluid1d;

import java.util.ArrayList;
import maspack.matrix.*;

public class CenterlineLinear extends Centerline
{

    public void resample(double ds)
    {  
        ArrayList<Point3d> newPoints = new ArrayList<Point3d>((int)(length/ds)+1);

        newPoints.add(points.get(0));			// add the previous startpoint
        double sCurr = 0.0 + ds;
        while (sCurr < length)
        {
            newPoints.add(getWorldLocation(sCurr));
            sCurr = sCurr + ds;
        }
        newPoints.add(points.get(points.size()-1));	// add the previous endpoint

        // if the last two points are very close, remove the second to last one
        int L = newPoints.size();
        if ( (newPoints.get(L-1).distance(newPoints.get(L-2))) < (newPoints.get(L-1).distance(newPoints.get(L-3)))/100.0 )
        {
            newPoints.remove(L-2);
        }

        setVertices(newPoints);
    }

    public void resample(int nSegments)
    {
        resample(length/((double)nSegments));
    }

    public Vector3d findDirectionAtVertex(int vertexIndex)
    {
        if ( vertexIndex == 0 )
            return findDirectionBetweenVertices(1,0);
        else if ( vertexIndex == nPoints-1 )
            return findDirectionBetweenVertices(nPoints-2,nPoints-1);
        else
            return findDirectionBetweenVertices(vertexIndex+1,vertexIndex-1);
    }

    public Vector3d findDirectionBetweenVertices(int v1, int v2)
    {
        // v1: vector head, v2: vector tail
        Vector3d dir = new Vector3d();
        dir.sub(points.get(v1), points.get(v2));
        dir.normalize();					// I think a unit vector is prefered...

        return dir;
    }

    public String getCenterlineName() 
    {
        return String.format("centerline_lin_%dpnts", nPoints);
    }

}
