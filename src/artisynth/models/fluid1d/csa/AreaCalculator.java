package artisynth.models.fluid1d.csa;


import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.*;

/*//
 * Author: petera
 * This class provides methods for calculating the cross-sectional area and perimeter for
 * ordered points (supposedly) lying in a cutplane.
 * TODO: only retain the static methods?
//*/


public class AreaCalculator 
{
    double area;
    double perimeter;
    Point3d centroid;

    Vector3d planeNormal;

    ArrayList<Point3d> points3d;
    ArrayList<Point3d> points2d;

    public enum AreaMethod{project2d, direct3d};
    AreaMethod areaMethod = AreaMethod.project2d;

    // optionally, define a static-plane, in which case the rotation matrix wouldn't change

    public void update()
    {
        if (areaMethod == AreaMethod.project2d)
        {
            if (planeNormal == null)
            {
                planeNormal = calcBestFitPlane(points3d);
            }

            RotationMatrix3d rot = calcWorldToPlaneRotation(planeNormal);

            points2d = new ArrayList<Point3d>(points3d.size());
            for (Point3d p : points3d)
            {
                Point3d pNew = new Point3d(p);
                pNew.transform (rot);
                points2d.add (pNew);
            }

            area = calcArea2d(points2d);
            perimeter = calcPerimeter2d(points2d);

        }
        else if (areaMethod == AreaMethod.direct3d)
        {
            Vector3d areaVector = calcAreaVector3d(points3d);
            if (planeNormal != null)
            {
                area = areaVector.dot(planeNormal);
            }
            else
            {
                area = areaVector.norm();
                areaVector.normalize();
                planeNormal = areaVector;
            }

            perimeter = calcPerimeter3d(points3d);
        }

        centroid = calcCentroidOfPoints(points3d);
    }

    public static Vector3d calcBestFitPlane(ArrayList<Point3d> points)
    {
        return calcBestFitPlane(points, 0);
    }

    public static Vector3d calcBestFitPlane(ArrayList<Point3d> points, int iter)
    {
        // find the best fit plane to a given point set; return the plane normal
        // a*x + b*y + c*z = d --> z = d/c - (a/c)*x - (b/c)*y = C0 + C1*x + C2*y
        int N = points.size();

        if (N<3)
            return null;		// 3 points define a plane, 4+ for a best-fit

        MatrixNd C = new MatrixNd();		// 3x1 --> the solution coefficients
        MatrixNd Z = new MatrixNd(N,1);		// the right hand side; z-values
        MatrixNd A = new MatrixNd(N,3);		// the Nx3 matrix

        for (int n=0; n<N; n++)
        {
            Point3d p = points.get(n);
            A.set(n,0, 1.0);
            A.set(n,1, p.x);
            A.set(n,2, p.y);
            Z.set(n,0, p.z);
        }

        // least squares solve: A^T*A*C = A^T*Z --> C = (A^T*A)^-1 * A^T*Z
        MatrixNd At = new MatrixNd();
        At.transpose(A);

        C.mul(At,A);
        //double det = C.determinant();
        C.invert();
        C.mul(At);
        C.mul(Z);

        // check for degenerate c=0 case, when plane is parallel to z-axis  
        //System.out.println(String.format("matrix quality: \t det=%f, \t C0=%f, \t C1=%f, \t C2=%f", det, C.get(0,0), C.get(1,0), C.get(2,0)));
        //if ( C.containsNaN() == true )
        if ( (C.containsNaN() == true) || (Math.abs(C.get(0,0))<0.001) || (Math.abs(C.get(1,0))<0.001) || (Math.abs(C.get(2,0))<0.001))
        {
            if (iter > 25)
                return null;	// just incase a very degenerate case is given (such as a line)

            // rotate the points, solve again, un-rotate normal
            RigidTransform3d trans = new RigidTransform3d(new Vector3d(1.0,1.0,1.0), new AxisAngle(1.0,1.0,1.0,Math.PI/4.0));
            ArrayList<Point3d> pRot = new ArrayList<Point3d>(N);
            for (Point3d p : points)
            {
                Point3d pNew = new Point3d(p.x, p.y, p.z);
                pNew.transform(trans);
                pRot.add(pNew);
            }
            iter++;
            Vector3d norm = calcBestFitPlane(pRot, iter);
            norm.inverseTransform(trans);
            return norm;
        }
        else
        {
            // solve for 3 points on the plane...
            double[] p0 = { 0.0, 0.0, 0.0};
            double[] p1 = { 1.0, 1.0, 0.0};
            double[] p2 = {-1.0, 1.0, 0.0};
            p0[2] = C.get(0,0) + C.get(1,0)*p0[0] + C.get(2,0)*p0[1];
            p1[2] = C.get(0,0) + C.get(1,0)*p1[0] + C.get(2,0)*p1[1];
            p2[2] = C.get(0,0) + C.get(1,0)*p2[0] + C.get(2,0)*p2[1];
            // create 2 vectors on the plane...
            Vector3d v1 = new Vector3d(p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]);
            Vector3d v2 = new Vector3d(p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]);
            // find the normal by taking the cross-product
            Vector3d norm = new Vector3d();
            norm.cross(v1,v2);
            norm.normalize();

            return norm;
        }

    }


    public static RotationMatrix3d calcWorldToPlaneRotation(Vector3d planeNormal)
    {
        // Rotate from world coordinates to plane coordinates where x',y' are in the plane and z' is the plane normal
        Vector3d c = new Vector3d(planeNormal);               // c = z' = plane normal
        Vector3d b = new Vector3d(1.0, 1.0, 0.0);
        Vector3d a = new Vector3d(0.0, 1.0, 1.0);
        a.normalize();
        b.normalize();
        c.normalize();
        // b = y' and a = x' --> can be arbitrary for now, but must be independant
        if (Math.abs(b.dot(c)) > 0.98)
            b.z = b.z + 1.0;
        if (Math.abs(a.dot(c)) > 0.98)
            a.z = a.z + 1.0;

        Vector3d A = new Vector3d(a);
        Vector3d B = new Vector3d(b);
        Vector3d C = new Vector3d(c);

        // perform Gram-Schmidt
        //C = c;                                                  // the direction of c does not get modified
        B.sub (B, GramSchmidtHelper(C,b));                        // make B orthogonal to C
        A.sub (A, GramSchmidtHelper(C,a));                        // make A orthogonal to C and B
        A.sub (A, GramSchmidtHelper(B,a));
        // make orthogonal
        A.normalize();
        B.normalize();
        C.normalize();

        RotationMatrix3d rot = new RotationMatrix3d();
        rot.setColumn (0, A);
        rot.setColumn (1, B);
        rot.setColumn (2, C);
        rot.transpose ();

        return rot;
    }

    static Vector3d GramSchmidtHelper(Vector3d v1, Vector3d v2)
    {
        // returns v1'*v2/(v1'*v1)*v1
        Vector3d res = new Vector3d();
        res.scale ((v1.dot(v2))/(v1.dot(v1)), v1);
        return res;
    }

    public static RigidTransform3d calcWorldToPlaneTransformation(Vector3d normal, Point3d origin)
    {
        // this calculates the transform that will bring a plane with the given normal and origin
        // to have the origin at 0,0,0 and the normal to be 0,0,1

        RotationMatrix3d rotMeshToWorld = AreaCalculator.calcWorldToPlaneRotation( normal );
        rotMeshToWorld.invert();							// now is mesh to world
        RigidTransform3d rt = new RigidTransform3d(origin, rotMeshToWorld);	// describes mesh to world
        rt.invert();								// now is worldToMesh

        //      RigidTransform3d rt = new RigidTransform3d();
        //      rt.setTranslation(origin);
        //      rt.setRotation(calcWorldToPlaneRotation(normal));
        return rt;
    }

    public static double calcArea2d(ArrayList<Point3d> points)
    {
        // the points should be ordered (clockwise or counterclockwise) for this calculation to be correct
        // also, it is assumed that the points describe a planar polygon in the x-y plane...if not transform to x-y plane before calling this
        double area = 0.0;
        Point3d pnt1;
        Point3d pnt2;

        for (int a=0; a<points.size (); a++)
        {
            pnt1 = points.get (a);
            if (a == points.size()-1)
                pnt2 = points.get (0);
            else
                pnt2 = points.get (a+1);

            area = area + (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
        }
        area = area*0.5;

        return area;
    }

    public static double calcArea3d(ArrayList<Point3d> points)
    {
        Vector3d areaVec = calcAreaVector3d(points);

        return areaVec.norm();
    }

    public static Vector3d calcAreaVector3d(ArrayList<Point3d> points)
    {
        //double area = 0.0;
        Point3d p1;
        Point3d p2;

        Vector3d sum = new Vector3d();

        for (int a=0; a<points.size (); a++)
        {
            p1 = points.get (a);
            if (a == points.size()-1)
                p2 = points.get (0);
            else
                p2 = points.get (a+1);

            Vector3d cp = new Vector3d();
            cp.cross(p1, p2);
            sum.add(cp);
        }
        sum.scale(0.5);

        return sum;
    }

    public static double calcPerimeter2d(ArrayList<Point3d> points)
    {
        int N = points.size();
        double perim = 0.0;  
        Point3d p1;
        Point3d p2;

        for (int a=0; a<N; a++)
        {
            p1 = points.get(a);
            if (a == N-1)
                p2 = points.get(0);
            else
                p2 = points.get(a+1);

            perim = perim + Math.sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );

        }
        return perim;
    }

    public static double calcPerimeter3d(ArrayList<Point3d> points)
    {
        int N = points.size();
        double perim = points.get(0).distance(points.get(N-1));
        for (int a=1; a<N; a++)
        {
            perim = perim + points.get(a).distance(points.get(a-1));
        }
        return perim;
    }

    public static Point3d calcCentroidOfPoints(ArrayList<Point3d> points)
    {
        Point3d centroid = new Point3d();
        for (Point3d p : points) 
        {
            centroid.add (p);
        }
        centroid.scale (1.0 / (double)points.size());

        return centroid;
    }
    // ------------------------------------//

    public static boolean isPolylineClosed(Polyline pl)
    {
        //if (pl.getVertex(0).uniqueIndex == pl.getVertex(pl.numVertices ()-1).uniqueIndex)
        if (pl.getVertex(0).pnt.distance(pl.getVertex(pl.numVertices ()-1).pnt) < 1e-10)
            return true;
        else
            return false;
    }

    public static boolean isPointInsidePolyline(Point3d point, Polyline pl)
    {
        //return isPointInsidePolyline_WindingAngle(point, pl);
        return isPointInsidePolyline_RayCast(point, pl);
    }

    public static boolean isPointInsidePolyline_WindingAngle(Point3d point, Polyline pl)
    {
        // The assumption is that the point and polygon are in cutplane coordinates, with x,y lying in plane and z normal to plane
        int nPoints = pl.numVertices();
        Vector3d v1;
        Vector3d v2;
        Vector3d r = new Vector3d();
        double theta;
        double thetaSum = 0.0;
        for (int a=0; a<nPoints-1; a++)
        {
            v1 = new Vector3d(pl.getVertex(a  ).pnt.x - point.x, pl.getVertex(a  ).pnt.y - point.y, pl.getVertex(a  ).pnt.z - point.z);
            v2 = new Vector3d(pl.getVertex(a+1).pnt.x - point.x, pl.getVertex(a+1).pnt.y - point.y, pl.getVertex(a+1).pnt.z - point.z);
            r.cross(v1, v2);
            theta = Math.acos( v1.dot(v2)/(v1.norm()*v2.norm()) );
            if (r.z >= 0.0)
                theta = theta*1.0;
            else
                theta = theta*-1.0;
            thetaSum = thetaSum + theta;
        }
        // if thetaSum is nearly equal to an integer multiple of 2*pi, then the point is inside. 
        double cF = thetaSum/(Math.PI*2.0);
        int cI = (int)Math.round(cF); 
        if (Math.abs(cF-(double)cI) < 1e-5)
            return true;
        else
            return false;
    }

    public static boolean isPointInsidePolyline_RayCast(Point3d point, Polyline pl)
    {
        // I don't trust this calculation. Perhaps because the Edge-Edge intersection code is not robust for degenerate cases.
        // This calculation is rightly for a 2d point and 2d polygon

        int nInts = 0;    // num intersections: even --> point is outside, odd --> point is inside
        Point3d testPoint;
        Point3d pMin = new Point3d();
        Point3d pMax = new Point3d();
        pl.updateBounds (pMin, pMax);
//        Point3d outsidePoint = new Point3d();
//        outsidePoint.scaledAdd(2.0, pMax, pMin);
        //add (pMax, pMax);   
        Point3d outsidePoint = new Point3d(); // ensure that outsidePoint is indeed outside polygon
        outsidePoint.scale(2.0, pMax);
        outsidePoint.sub(pMin);

        for (int n=1; n<pl.numVertices (); n++)
        {
            testPoint = edgeEdgeIntersection2d (point, outsidePoint, pl.getVertex(n).getPosition(), pl.getVertex(n-1).getPosition() );
            if (testPoint == null)
                ;   // no intersection, do nothing
            else
                nInts++;
        }
        if (nInts % 2 == 0)
            return false;
        else
            return true;
    }

    public static boolean isPolylineInsidePolyline(Polyline plTest, Polyline plMain)
    {
        // check if plTest is internal to plMain (requiring each point of plTest to be within plMain)
        for (int n=0; n<plTest.numVertices (); n++)
        {
            if (isPointInsidePolyline (plTest.getVertex(n).getPosition(), plMain) == false)
                return false;
        }
        return true;
    }

    public static Point3d edgeEdgeIntersection2d_old(Point3d e1p1, Point3d e1p2, Point3d e2p1, Point3d e2p2 )
    {
        // Note: the z-coordinate is ignored here...really just a Point2d
        double m1 = (e1p2.y - e1p1.y)/(e1p2.x - e1p1.x);
        double m2 = (e2p2.y - e1p1.y)/(e2p2.x - e1p1.x);
        if (m1 == m2)
            return null;   // edges are parallel

        double b1 = (e1p1.y - m1*e1p1.x);
        double b2 = (e2p1.y - m2*e2p1.x);
        double xInt = (b2 - b1)/(m1-m2);
        double yInt = m1*xInt + b1;

        return new Point3d(xInt, yInt, 0.0);
    }

    public static Point3d edgeEdgeIntersection2d(Point3d e1p1, Point3d e1p2, Point3d e2p1, Point3d e2p2 )
    {
        // the intersection is calculated in the x-y plane, but the 3d point is returned
        double a1 = e1p2.x - e1p1.x;
        double b1 = e1p2.y - e1p1.y;
        double c1 = e1p2.z - e1p1.z;
        double a2 = e2p2.x - e2p1.x;
        double b2 = e2p2.y - e2p1.y;
        double c2 = e2p2.z - e2p1.z;

        // of course I could just solve this analytically...
        int N = 2;
        MatrixNd x = new MatrixNd(N,1);
        MatrixNd b = new MatrixNd(N,1);
        MatrixNd A = new MatrixNd(N,N);
        A.set(0,0,  a1);
        A.set(0,1, -a2);
        A.set(1,0,  b1);
        A.set(1,1, -b2);
        b.set(0,0, e2p1.x - e1p1.x);
        b.set(1,0, e2p1.y - e1p1.y);
        MatrixNd Ainv = new MatrixNd();
        Ainv.invert(A);
        x.mul(Ainv,b);

        double t1 = x.get(0,0);
        double t2 = x.get(1,0);

        //      double m1 = (e1p2.y - e1p1.y)/(e1p2.x - e1p1.x);
        //      double m2 = (e2p2.y - e2p1.y)/(e2p2.x - e2p1.x);
        //      if (m1 == m2)
        //       return null;   // edges are parallel
        //      
        ////      double b1 = (e1p1.y - m1*e1p1.x);
        ////      double b2 = (e2p1.y - m2*e2p1.x);
        //      double xInt = ((e2p1.y - m2*e2p1.x) - (e1p1.y - m1*e1p1.x))/(m1-m2);
        //      double yInt = m1*xInt + (e1p1.y - m1*e1p1.x);
        //      double t1;
        //      double t2;
        //      if (a1 == 0.0)
        //       t1 = (yInt - e1p1.y)/b1;
        //      else
        //       t1 = (xInt - e1p1.x)/a1;
        //      if (a2 == 0.0)
        //       t2 = (yInt - e2p1.y)/b2;
        //      else
        //       t2 = (xInt - e2p1.x)/a2;

        // a parametric solve for where line1 and line2 cross
        //      double t2 = ( (e2p1.y - e1p1.y) - (e2p1.x - e1p1.x) )/(a2/a1 - b2/b1);
        //      double t1 = (e2p1.x - e1p1.x) + (a2/a1)*t2;

        // 0.0 <= t1 <= 1.0 and 0.0 <= t2 <= 1.0 for the *edges* to intersect
        //if ( (t1 > 1.0) || (t2 > 1.0) || (t1 < 0.0) || (t2 < 0.0) )
        // I need to look into these calcs, but seems like t1,t2 are NaN at some times of no intersection 
        if ( (t1 > 1.0) || (t2 > 1.0) || (t1 < 0.0) || (t2 < 0.0) || (Double.isNaN(t1)) || (Double.isNaN(t2)) )
            return null;

        return new Point3d(e1p1.x + a1*t1, e1p1.y + b1*t1, e1p1.z + c1*t1);
    }

    public static PolylineMesh findSlice_KeepLargest(PolylineMesh slice, Point3d centerPoint) 
    {
        return findSlice_KeepLargest(slice, centerPoint, false);
    }
    
    public static PolylineMesh findSlice_KeepLargest(PolylineMesh slice, Point3d centerPoint, boolean openAllowed) 
    {
        PolylineMesh areaMesh = new PolylineMesh();
        areaMesh.setMeshToWorld(slice.getMeshToWorld());

        Polyline plMain = null;
        double areaMax = 0.0;
        
        //int iMainCont = -1;
        int numContours = slice.numLines();
        for (int n=0; n<numContours; n++)
        {
            Polyline pl = slice.getLines().get(n);
            if ( (isPolylineClosed(pl) == true) || (openAllowed == true) )
            {
                ArrayList<Point3d> points = new ArrayList<Point3d>(pl.numVertices());
                for (Vertex3d vert : pl.getVertices())
                    points.add(vert.getPosition());
                
                double area = calcArea3d(points);
                if (area > areaMax)
                {
                    areaMax = Math.abs(area);
                    plMain = pl;
                }
            }
        }
        
        areaMesh.addLine(plMain);
        return areaMesh;
    }
    
    public static PolylineMesh findSlice_Contours(PolylineMesh slice, Point3d centerPoint) 
    {
        PolylineMesh areaMesh = new PolylineMesh();
        areaMesh.setMeshToWorld(slice.getMeshToWorld());

        // find the main contour: the innermost contour which the centerpoint lies inside of
        int iMainCont = -1;
        int numContours = slice.numLines();
        for (int n=0; n<numContours; n++)
        {
            Polyline pl = slice.getLines().get(n);
            if (isPolylineClosed(pl) == true)
            {
                if (isPointInsidePolyline(centerPoint, pl) == true)
                {
                    if (areaMesh.numLines() == 0)
                    {
                        areaMesh.addLine(pl);
                        iMainCont=n;
                    }
                    else if ( isPolylineInsidePolyline(pl, areaMesh.getLines().get(0)) == true )
                    {
                        //areaMesh.getLines().remove(0);
                        areaMesh.getLines().clear();
                        areaMesh.addLine(pl);
                        iMainCont=n;
                    }
                }
            }
        }
        // find any island contours that are inside of the main contour but external to the centerpoint
        // TODO: to speed this code up, keep a list of rejected contours (open, outside of centerpoint, etc...)
        if (areaMesh.numLines() > 0)
        {
            // find any internal contours (contours located inside the main contour, but not containing the centerline point)
            for (int n=0; n<numContours; n++)
            {
                if (n != iMainCont)
                {
                    Polyline pl = slice.getLines().get(n);
                    if (isPolylineClosed(pl) == true)
                    {
                        if (isPolylineInsidePolyline(pl, areaMesh.getLines().get(0)) == true)
                        {
                            areaMesh.addLine(pl);
                        }
                    }
                }
            }
        }
        return areaMesh;
    }

    public static PolylineMesh findVisibleSlice_2dExact(PolylineMesh slice, Point3d centerPoint) 
    {
        /*//
         * We are looking to reduce a complete cross-sectional slice of a geometry, to a slice of interest. 
         * In this case, the slice-of-interest is defined by what is visible to the centerpoint.
         * 
         * This calculation is 2D, working in the "cutplane" coordinates (everything lies in the x-y plane. 
         * The centerPoint is expected to be in these coordinates, and the slice is expected to have 
         * local (mesh) coordinates in this system.    
      //*/

        PolylineMesh sliceOI = new PolylineMesh();                      // slice-of-interest
        RigidTransform3d xMeshToWorld = slice.getMeshToWorld();           // transform between cutplane and world coords

        // 3) calculate the visible slice
        ArrayList<Point3d> visiblePoints = new ArrayList<Point3d>();      // vis points in plane coords
        for (int a=0; a<slice.numVertices(); a++)
        {
            Point3d point = slice.getVertices().get(a).getPosition();      // working in mesh (plane) coords
            // one edge is insidePoint <--> point

            boolean isVisible = true;
            edgesLoop:
                for ( Polyline pl : slice.getLines() )
                {
                    for (int b=0; b<pl.numVertices()-1; b++)
                    {
                        // edge-edge intersection
                        Point3d pInt = edgeEdgeIntersection2d(centerPoint, point, pl.getVertex(b).pnt, pl.getVertex(b+1).pnt );
                        if ( (pInt != null) && (pInt.distance(point)> 1e-10) ) // might be a dangerous check
                            //if ( pInt != null )
                        {
                            isVisible = false;
                            break edgesLoop;
                        }
                    }
                }

            if (isVisible == true)
                visiblePoints.add(point);    
        }

        ArrayList<Point3d> orderedPoints = order2dPointsByAngle(visiblePoints, centerPoint);
        int nVisPoints = orderedPoints.size();
        Vertex3d[] verts = new Vertex3d[nVisPoints+1];
        for (int a=0; a<nVisPoints; a++)
        {
            Vertex3d v = sliceOI.addVertex(orderedPoints.get(a));
            verts[a] = v;
        }
        verts[nVisPoints] = verts[0];
        sliceOI.addLine(verts);
        sliceOI.setMeshToWorld(xMeshToWorld);

        return sliceOI;

    }

    public static ArrayList<Point3d> order2dPointsByAngle(ArrayList<Point3d> points, Point3d insidePoint)
    {
        // the incoming points are assumed to be in planar coordinates; this calculations doesn't make sense in 3d
        int nPoints = points.size();
        double[] thetas = new double[nPoints];
        //int[] order = new int[nPoints];         // records indices of points, from smallest theta to largest
        ArrayList<Point3d> orderedPoints = new ArrayList<Point3d>(nPoints);

        for (int a=0; a<nPoints; a++)
        {
            thetas[a] = Math.atan2(points.get(a).y - insidePoint.y, points.get(a).x - insidePoint.x);

            if (a==0)
            {
                orderedPoints.add(points.get(a));
            }
            else
            {
                int b=a;
                while (thetas[b] < thetas[b-1])
                {
                    double temp = thetas[b-1];
                    thetas[b-1] = thetas[b];
                    thetas[b] = temp;
                    b--;
                    if (b==0)
                        break;
                }
                orderedPoints.add(b, points.get(a));
            }

        }
        return orderedPoints;
    }

    //*//
    // TODO: fix this code. Ideally, input a geometry and dTheta, and return a set of ordered points which are visible to the center point.
    public static  PolylineMesh findVisibleSlice_3dRays(PolylineMesh slice, Point3d insidePoint, PolygonalMesh mesh) 
    {
        // Choose only the points in the slice that are visible to insidePoint
        RigidTransform3d meshToWorld = slice.getMeshToWorld();
        
        Point3d insidePointWorld = new Point3d(insidePoint); 
        insidePointWorld.transform(meshToWorld);
        
        PolylineMesh sliceWorld = new PolylineMesh(slice); // make a copy
        sliceWorld.setMeshToWorld(new RigidTransform3d() );
        sliceWorld.transform(meshToWorld);
        
        mesh.updateFaceNormals();
        //mesh.computedFaceNormals();

        // 1) find only the points that can see the "inside point"
        ArrayList<Point3d> visiblePoints = new ArrayList<Point3d>();
        for (Vertex3d vert : sliceWorld.getVertices())
        {
            Point3d point = vert.getPosition();
            Vector3d rayDirC2F = new Vector3d();
            rayDirC2F.sub(point, insidePointWorld);                     // vector from centerline to face
            rayDirC2F.normalize();
            //rayDirC2F.scale(0.75);
//            Vector3d rayDirF2C = rayDirC2F.clone();
//            rayDirF2C.scale(-1.0);
            //rayDirC2F.normalize();

            Point3d t1 = BVFeatureQuery.nearestPointAlongRay(mesh, insidePointWorld, rayDirC2F);
            //Point3d t2 = BVFeatureQuery.nearestPointAlongRay(mesh, insidePointWorld, rayDirF2C);
            //Point3d t1 = nearestPointIntersectedByRay(mesh, insidePoint, rayDirC2F);
            //Point3d t2 = nearestPointIntersectedByRay(mesh, insidePoint, rayDirF2C);
            Point3d intPoint = t1;         // I think it *should* be t1!
            if (intPoint == null)
            {
                visiblePoints.add(point); // this case should not happen!
            }
            else if (point.distance(intPoint) < 1e-9)
            {
                visiblePoints.add(point);
            }
        }

        // now I need to transform the points back to mesh coordinates
        RigidTransform3d worldToMesh = new RigidTransform3d(meshToWorld);
        worldToMesh.invert();
        for (Point3d p : visiblePoints)
            p.transform(worldToMesh);

        // valid points to ordered points (same for 2d calculation)
        PolylineMesh sliceOI = new PolylineMesh();
        ArrayList<Point3d> orderedPoints = order2dPointsByAngle(visiblePoints, insidePoint);
        int nVisPoints = orderedPoints.size();
        Vertex3d[] verts = new Vertex3d[nVisPoints+1];
        for (int a=0; a<nVisPoints; a++)
        {
            Vertex3d v = sliceOI.addVertex(orderedPoints.get(a));
            verts[a] = v;
        }
        verts[nVisPoints] = verts[0];
        sliceOI.addLine(verts);
        sliceOI.setMeshToWorld(slice.getMeshToWorld());
        return sliceOI;
            
            //     if (t1.distance(t2) > 0.000001)
            //     {
            //        // ray direction matters --> this should always be the case
            //        int temp = 1;
            //     }
            //     else
            //     {
            //        // ray direction doesn't matter --> grrrr!
            //        int temp = 1;
            //     }
            //     
            //
            //     Intersector intersector = new Intersector();           // what is this?
            //     Vector3d duv = new Vector3d();                         // what is this? does it need to be new for each collision?
            //
            //     //Vector3d nearest = new Vector3d();
            //     //Face faceHit = mesh.getBvHierarchy().nearestFaceToPoint(point, nearest, new Point3d(), new Vector2d(), new Intersector());
            //     // TODO: this faceHit method below seems to be completely wrong.  Why?
            //     Face faceHit = mesh.getBvHierarchy().nearestFaceIntersectedByRay(insidePoint, rayDirC2F, duv, intersector); // --> calc ray to face, then use this
            //     if (faceHit != null)   // theoretically, this should never be null...
            //     {
            //        Point3d wCent = new Point3d();
            //        faceHit.computeWorldCentroid(wCent);
            //        Plane plane = new Plane(faceHit.getWorldNormal(), wCent);
            //        Point3d iSect = new Point3d();
            //        double dist = plane.intersectLine(iSect, rayDirC2F, insidePoint);
            //        if (Math.abs(point.norm() - iSect.norm()) < 0.000001)
            //        {
            //           visiblePoints.add(point);
            //        }
            //     }
            //     else
            //     {
            //        // these points *should* hit something; if they numerically miss, they are included
            //        // TODO: is this a safe assumption always??
            //        visiblePoints.add(point);
            //     }

//        PolylineMesh sliceVis = new PolylineMesh();
//        for (Point3d p : visiblePoints)
//            sliceVis.addVertex(p);
        //slice.setWorldPoints(visiblePoints);
        //slice.setPlane(sliceOrigin, sliceNormal);
        //slice.calculatePlanePoints();
        //slice.orderPointsByAngle(insidePoint);
    }
    //*/

    public static Point3d nearestPointIntersectedByRay(PolygonalMesh mesh, Point3d rayOrigin, Vector3d rayDir)
    {
        return nearestPointIntersectedByRay(mesh, rayOrigin, rayDir, new Face(0), 0.0);
    }

    public static Point3d nearestPointIntersectedByRay(PolygonalMesh mesh, Point3d rayOrigin, Vector3d rayDir, Face intersectedFace, double dist)
    {
//        Point3d nearPnt = BVFeatureQuery.nearestPointAlongRay(mesh, rayOrigin, rayDir);
        
        BVFeatureQuery query = new BVFeatureQuery();
        Vector3d duv = new Vector3d();                            // what is this? does it need to be new for each collision?
        

        intersectedFace = query.nearestFaceAlongRay (null, duv, mesh, rayOrigin, rayDir);
        if (intersectedFace != null)
        {
            Point3d wCent = new Point3d();
            intersectedFace.computeWorldCentroid(wCent);
            Plane plane = new Plane(intersectedFace.getWorldNormal(), wCent);
            Point3d iSect = new Point3d();
            dist = plane.intersectLine(iSect, rayDir, rayOrigin);
            return iSect;
        }
        else
        {
            return null;
        }
    }


    // --- some getters and setters --- //

    public void setCalculationMethod(AreaMethod am)
    {
        areaMethod = am;
    }

    public void setPoints(ArrayList<Point3d> points)
    {
        // these points are assumed to be properly ordered!
        points3d = points;
    }

    public void setPlaneNormal(Vector3d normal)
    {
        normal.normalize();		// just to be safe...
        planeNormal = normal;
    }

    public double getArea()
    {
        return area;
    }

    public double getPerimeter()
    {
        return perimeter;
    }

    public Vector3d getPlaneNormal()
    {
        return planeNormal;
    }


}
