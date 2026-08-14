package artisynth.models.fluid1d.csa;

import java.util.ArrayList;


import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

/*//
 * Author: petera
 * 
 * The purpose of this class is to:
 * 1) take a slice through an arbitrary (possibly quite messy) geometry and determine the relevant cross-sectional area by either:
 * 1.1) exact calculations (which require closed contours), or
 * 1.2) line-of-sight calculations (either 2D or 3D methods are possible)
 * 2) track the relevant cross-sectional area, either through:
 * 2.1) re-slicing and re-interpreting each time-step, or
 * 2.2) adding the ordered contour points as markers and tracking those
 * 3) calculating the area and perimeter
 * 
 * Input:
 * 1) the point and normal of cutplane --or-- pre-defined points that define cutplane
 * 2) the geometry(s) through which to calculate the slice
 * Output:
 * A polyline mesh containing the relevant contours.
//*/

public abstract class SliceInterpreter
{
    String name = "slice";
    public double area;
    public double perimeter;

    boolean findSlice = true;
    boolean sliceEachStep = false;		// TODO: not working with the skin mesh
    boolean use3dAreaCalcs = true;

    SliceGeometry gs = new SliceGeometry();
    public enum OrderingMethod {ClosedContours, LineOfSight_2dExact, LineOfSight_3dRays, KeepLargest}; // , LineOfSight_3dRays        
    OrderingMethod orderingMethod = OrderingMethod.LineOfSight_2dExact;       // ClosedContours is exact, but requires a watertight geom

    // The first polyline (polygon) is the main contour which contains the centerpoint. 
    // Additional lines describe internal "island" areas to subtract out. 
    public PolylineMesh areaMesh;                       

    public abstract void initialize();
    //public abstract void update();
    public abstract void attachPointsAsMarkers();

    public SliceInterpreter()
    {
    }

    public SliceInterpreter(String name, Point3d planeCenter, Vector3d planeNormal)
    {
        setName(name);
        setCutplaneByPointAndNormal(planeCenter, planeNormal);
    }

    public SliceInterpreter(String name, ArrayList<Point3d> points)
    {
        setName(name);
        setCutplaneByPoints(points);
    }

    public void update()
    {
        // many options from the area calculator could be added here if needed...
        if (findSlice == true)
        {
            gs.update();   // update the slice of the entire geometry
            //RigidTransform3d transform = AreaCalculator.calcWorldToPlaneTransformation(sliceNormal, sliceOrigin);
            Point3d pointXY = new Point3d( gs.getPlaneOrigin() );      
            pointXY.inverseTransform( gs.slice.getMeshToWorld() );

            if (orderingMethod == OrderingMethod.ClosedContours)
            {
                // find the main contour (the innermost contour which contains the centerline point
                areaMesh = AreaCalculator.findSlice_Contours(gs.slice, pointXY);
            }
            else if (orderingMethod == OrderingMethod.LineOfSight_2dExact)
            {   
                areaMesh = AreaCalculator.findVisibleSlice_2dExact(gs.slice, pointXY);
            }
            else if (orderingMethod == OrderingMethod.LineOfSight_3dRays)
            {
                areaMesh = AreaCalculator.findVisibleSlice_3dRays(gs.slice, pointXY, gs.geoms.get(0)); // TODO: not OK
            }
            else if (orderingMethod == OrderingMethod.KeepLargest)
            {
                areaMesh = AreaCalculator.findSlice_KeepLargest(gs.slice, pointXY);
            }
            updateArea();

            if (sliceEachStep == false)
            {
                findSlice = false;
                attachPointsAsMarkers();
            }
        }
        else
        {
            updateArea();
        }
    }

    public void updateArea()
    {
        for (int a=0; a<areaMesh.numLines(); a++)
        {
            ArrayList<Point3d> points = new ArrayList<Point3d>();
            Polyline pl = areaMesh.getLines().get(a);
            
            double areaP, perimP;
            if (use3dAreaCalcs == true)
            {
                for (int b=0; b<pl.numVertices(); b++)
                    points.add(pl.getVertex(b).getWorldPoint());
                
                areaP  = AreaCalculator.calcArea3d(points);
                perimP = AreaCalculator.calcPerimeter3d(points);
            }
            else
            {
                for (int b=0; b<pl.numVertices(); b++)
                    points.add(pl.getVertex(b).getPosition());
                
                areaP  = Math.abs(AreaCalculator.calcArea2d(points));
                perimP = AreaCalculator.calcPerimeter2d(points);
            }

            if (a==0)
            {
                area = areaP;
                perimeter = perimP;
            }
            else
            {
                area = area - areaP;                // an internal area reduces the cross-sectional area...
                perimeter = perimeter + perimP;     // ...yet increases the wetted perimeter.
            }
        }
    }


    public void setName(String name)
    {
        this.name = name;
    }

    public void setCutplaneByPointAndNormal(Point3d planeCenter, Vector3d planeNormal)
    {
        gs.setPlane(planeCenter, planeNormal);
        findSlice = true;
    }

    public void setCutplaneByPoints(ArrayList<Point3d> points)
    {
        areaMesh = new PolylineMesh();
        int nVerts = points.size ();
        Vertex3d[] verts = new Vertex3d[nVerts];
        for (int n=0; n<nVerts; n++)
            verts[n] = new Vertex3d(points.get(n));
        areaMesh.addLine(verts);
        //this.mainContour = points;
        findSlice = false;
    }

    public void setSliceEachStep(boolean sliceEachStep)
    {
        this.sliceEachStep = sliceEachStep;
    }

    public void setOrderingMethod(OrderingMethod om)
    {
        this.orderingMethod = om;
    }
    
    public void setUse3dAreaCalcs(boolean use3d)
    {
        this.use3dAreaCalcs = use3d;
    }

    public PolylineMesh getRawSlice()
    {
        return gs.slice;
    }

    public PolylineMesh getSlice()
    {
        return areaMesh;
    }
    
    public PolygonalMesh getCutplane()
    {
        return gs.getCutplane();
    }


    // -- Below are calculations for interpreting a slice -- //

    //   public static boolean isPolylineClosed(Polyline pl)
    //   {
    //      //if (pl.getVertex(0).uniqueIndex == pl.getVertex(pl.numVertices ()-1).uniqueIndex)
    //      if (pl.getVertex(0).pnt.distance(pl.getVertex(pl.numVertices ()-1).pnt) < 1e-10)
    //         return true;
    //      else
    //         return false;
    //   }
    //   
    //   public static boolean isPointInsidePolyline(Point3d point, Polyline pl)
    //   {
    //      return isPointInsidePolyline_WindingAngle(point, pl);
    //   }
    //   
    //   public static boolean isPointInsidePolyline_WindingAngle(Point3d point, Polyline pl)
    //   {
    //      // The assumption is that the point and polygon are in cutplane coordinates, with x,y lying in plane and z normal to plane
    //      int nPoints = pl.numVertices();
    //      Vector3d v1;
    //      Vector3d v2;
    //      Vector3d r = new Vector3d();
    //      double theta;
    //      double thetaSum = 0.0;
    //      for (int a=0; a<nPoints-1; a++)
    //      {
    //         v1 = new Vector3d(pl.getVertex(a  ).pnt.x - point.x, pl.getVertex(a  ).pnt.y - point.y, pl.getVertex(a  ).pnt.z - point.z);
    //         v2 = new Vector3d(pl.getVertex(a+1).pnt.x - point.x, pl.getVertex(a+1).pnt.y - point.y, pl.getVertex(a+1).pnt.z - point.z);
    //         r.cross(v1, v2);
    //         theta = Math.acos( v1.dot(v2)/(v1.norm()*v2.norm()) );
    //         if (r.z >= 0.0)
    //            theta = theta*1.0;
    //         else
    //            theta = theta*-1.0;
    //         thetaSum = thetaSum + theta;
    //      }
    //      // if thetaSum is nearly equal to an integer multiple of 2*pi, then the point is inside. 
    //      double cF = thetaSum/(Math.PI*2.0);
    //      int cI = (int)Math.round(cF); 
    //      if (Math.abs(cF-(double)cI) < 1e-5)
    //         return true;
    //      else
    //         return false;
    //   }
    //   
    //   public static boolean isPointInsidePolyline_RayCast(Point3d point, Polyline pl)
    //   {
    //      // I don't trust this calculation. Perhaps because the Edge-Edge intersection code is not robust for degenerate cases.
    //      // This calculation is rightly for a 2d point and 2d polygon
    //      
    //      int nInts = 0;    // num intersections: even --> point is outside, odd --> point is inside
    //      Point3d testPoint;
    //      Point3d pMin = new Point3d();
    //      Point3d pMax = new Point3d();
    //      pl.updateBounds (pMin, pMax);
    //      Point3d outsidePoint = new Point3d();
    //      outsidePoint.add (pMax, pMax);   // ensure that outside point is beyond
    //      
    //      for (int n=1; n<pl.numVertices (); n++)
    //      {
    //         testPoint = edgeEdgeIntersection2d (point, outsidePoint, pl.getVertex(n).getPosition(), pl.getVertex(n-1).getPosition() );
    //         if (testPoint == null)
    //            ;   // no intersection, do nothing
    //         else
    //            nInts++;
    //      }
    //      if (nInts % 2 == 0)
    //         return false;
    //      else
    //         return true;
    //   }
    //   
    //   public static boolean isPolylineInsidePolyline(Polyline plTest, Polyline plMain)
    //   {
    //      // check if plTest is internal to plMain (requiring each point of plTest to be within plMain)
    //      for (int n=0; n<plTest.numVertices (); n++)
    //      {
    //         if (isPointInsidePolyline (plTest.getVertex(n).getPosition(), plMain) == false)
    //            return false;
    //      }
    //      return true;
    //   }
    //   
    //   public static Point3d edgeEdgeIntersection2d_old(Point3d e1p1, Point3d e1p2, Point3d e2p1, Point3d e2p2 )
    //   {
    //      // Note: the z-coordinate is ignored here...really just a Point2d
    //      double m1 = (e1p2.y - e1p1.y)/(e1p2.x - e1p1.x);
    //      double m2 = (e2p2.y - e1p1.y)/(e2p2.x - e1p1.x);
    //      if (m1 == m2)
    //         return null;   // edges are parallel
    //      
    //      double b1 = (e1p1.y - m1*e1p1.x);
    //      double b2 = (e2p1.y - m2*e2p1.x);
    //      double xInt = (b2 - b1)/(m1-m2);
    //      double yInt = m1*xInt + b1;
    //      
    //      return new Point3d(xInt, yInt, 0.0);
    //   }
    //   
    //   public static Point3d edgeEdgeIntersection2d(Point3d e1p1, Point3d e1p2, Point3d e2p1, Point3d e2p2 )
    //   {
    //      // the intersection is calculated in the x-y plane, but the 3d point is returned
    //      double a1 = e1p2.x - e1p1.x;
    //      double b1 = e1p2.y - e1p1.y;
    //      double c1 = e1p2.z - e1p1.z;
    //      double a2 = e2p2.x - e2p1.x;
    //      double b2 = e2p2.y - e2p1.y;
    //      double c2 = e2p2.z - e2p1.z;
    //      
    //      // of course I could just solve this analytically...
    //      int N = 2;
    //      MatrixNd x = new MatrixNd(N,1);
    //      MatrixNd b = new MatrixNd(N,1);
    //      MatrixNd A = new MatrixNd(N,N);
    //      A.set(0,0,  a1);
    //      A.set(0,1, -a2);
    //      A.set(1,0,  b1);
    //      A.set(1,1, -b2);
    //      b.set(0,0, e2p1.x - e1p1.x);
    //      b.set(1,0, e2p1.y - e1p1.y);
    //      MatrixNd Ainv = new MatrixNd();
    //      Ainv.invert(A);
    //      x.mul(Ainv,b);
    //      
    //      double t1 = x.get(0,0);
    //      double t2 = x.get(1,0);
    //      
    ////      double m1 = (e1p2.y - e1p1.y)/(e1p2.x - e1p1.x);
    ////      double m2 = (e2p2.y - e2p1.y)/(e2p2.x - e2p1.x);
    ////      if (m1 == m2)
    ////       return null;   // edges are parallel
    ////      
    //////      double b1 = (e1p1.y - m1*e1p1.x);
    //////      double b2 = (e2p1.y - m2*e2p1.x);
    ////      double xInt = ((e2p1.y - m2*e2p1.x) - (e1p1.y - m1*e1p1.x))/(m1-m2);
    ////      double yInt = m1*xInt + (e1p1.y - m1*e1p1.x);
    ////      double t1;
    ////      double t2;
    ////      if (a1 == 0.0)
    ////       t1 = (yInt - e1p1.y)/b1;
    ////      else
    ////       t1 = (xInt - e1p1.x)/a1;
    ////      if (a2 == 0.0)
    ////       t2 = (yInt - e2p1.y)/b2;
    ////      else
    ////       t2 = (xInt - e2p1.x)/a2;
    //      
    //      // a parametric solve for where line1 and line2 cross
    ////      double t2 = ( (e2p1.y - e1p1.y) - (e2p1.x - e1p1.x) )/(a2/a1 - b2/b1);
    ////      double t1 = (e2p1.x - e1p1.x) + (a2/a1)*t2;
    //      
    //      // 0.0 <= t1 <= 1.0 and 0.0 <= t2 <= 1.0 for the *edges* to intersect
    //      if ( (t1 > 1.0) || (t2 > 1.0) || (t1 < 0.0) || (t2 < 0.0) )
    //         return null;
    //      
    //      return new Point3d(e1p1.x + a1*t1, e1p1.y + b1*t1, e1p1.z + c1*t1);
    //   }
    //   
    //   public static PolylineMesh findSlice_Contours(PolylineMesh slice, Point3d centerPoint) 
    //   {
    //      PolylineMesh areaMesh = new PolylineMesh();
    //      areaMesh.setMeshToWorld(slice.getMeshToWorld());
    //      
    //      int iMainCont = -1;
    //      int numContours = slice.numLines();
    //      for (int n=0; n<numContours; n++)
    //      {
    //         Polyline pl = slice.getLines().get(n);
    //         if (isPolylineClosed(pl) == true)
    //         {
    //            if (isPointInsidePolyline(centerPoint, pl) == true)
    //            {
    //               if (areaMesh.numLines() == 0)
    //               {
    //                  areaMesh.addLine(pl);
    //                  iMainCont=n;
    //               }
    //               else if ( isPolylineInsidePolyline(pl, areaMesh.getLines().get(0)) )
    //               {
    //                  areaMesh.getLines().remove(0);
    //                  areaMesh.addLine(pl);
    //                  iMainCont=n;
    //               }
    //            }
    //         }
    //      }
    //      // TODO: to speed this code up, keep a list of rejected contours (open, outside of main, etc...)
    //      if (areaMesh.numLines() > 0)
    //      {
    //         // find any internal contours (contours located inside the main contour, but not containing the centerline point)
    //         for (int n=0; n<numContours; n++)
    //         {
    //            if (n != iMainCont)
    //            {
    //               Polyline pl = slice.getLines().get(n);
    //               if (isPolylineClosed(pl) == true)
    //               {
    //                  if (isPolylineInsidePolyline(pl, areaMesh.getLines().get(0)) == true)
    //                  {
    //                     areaMesh.addLine(pl);
    //                  }
    //               }
    //            }
    //         }
    //      }
    //      return areaMesh;
    //   }
    //   
    //   public static PolylineMesh findVisibleSlice_2dExact(PolylineMesh slice, Point3d centerPoint) 
    //   {
    //      /*//
    //       * We are looking to reduce a complete cross-sectional slice of a geometry, to a slice of interest. 
    //       * In this case, the slice-of-interest is defined by what is visible to the centerpoint.
    //       * 
    //       * This calculation is 2D, working in the "cutplane" coordinates (everything lies in the x-y plane. 
    //       * The centerPoint is expected to be in these coordinates, and the slice is expected to have 
    //       * local (mesh) coordinates in this system.    
    //      //*/
    //      
    //      PolylineMesh sliceOI = new PolylineMesh();                      // slice-of-interest
    //      RigidTransform3d xMeshToWorld = slice.getMeshToWorld();           // transform between cutplane and world coords
    //      
    //      // 3) calculate the visible slice
    //      ArrayList<Point3d> visiblePoints = new ArrayList<Point3d>();      // vis points in plane coords
    //      for (int a=0; a<slice.numVertices(); a++)
    //      {
    //         Point3d point = slice.getVertices().get(a).getPosition();      // working in mesh (plane) coords
    //         // one edge is insidePoint <--> point
    //         
    //         boolean isVisible = true;
    //         edgesLoop:
    //         for ( Polyline pl : slice.getLines() )
    //         {
    //            for (int b=0; b<pl.numVertices()-1; b++)
    //            {
    //               // edge-edge intersection
    //               Point3d pInt = edgeEdgeIntersection2d(centerPoint, point, pl.getVertex(b).pnt, pl.getVertex(b+1).pnt );
    //               if ( (pInt != null) && (pInt.distance(point)> 1e-10) ) // might be a dangerous check
    //               //if ( pInt != null )
    //               {
    //                  isVisible = false;
    //                  break edgesLoop;
    //               }
    //            }
    //         }
    //         
    //         if (isVisible == true)
    //            visiblePoints.add(point);    
    //      }
    //      
    //      ArrayList<Point3d> orderedPoints = order2dPointsByAngle(visiblePoints, centerPoint);
    //      int nVisPoints = orderedPoints.size();
    //      Vertex3d[] verts = new Vertex3d[nVisPoints+1];
    //      for (int a=0; a<nVisPoints; a++)
    //      {
    //         Vertex3d v = sliceOI.addVertex(orderedPoints.get(a));
    //         verts[a] = v;
    //      }
    //      verts[nVisPoints] = verts[0];
    //      sliceOI.addLine(verts);
    //      sliceOI.setMeshToWorld(xMeshToWorld);
    //      
    //      return sliceOI;
    //      
    //   }
    //   
    //   public static ArrayList<Point3d> order2dPointsByAngle(ArrayList<Point3d> points, Point3d insidePoint)
    //   {
    //      // the incoming points are assumed to be in planar coordinates; this calculations doesn't make sense in 3d
    //      int nPoints = points.size();
    //      double[] thetas = new double[nPoints];
    //      //int[] order = new int[nPoints];         // records indices of points, from smallest theta to largest
    //      ArrayList<Point3d> orderedPoints = new ArrayList<Point3d>(nPoints);
    //      
    //      for (int a=0; a<nPoints; a++)
    //      {
    //         thetas[a] = Math.atan2(points.get(a).y - insidePoint.y, points.get(a).x - insidePoint.x);
    //         
    //         if (a==0)
    //         {
    //            orderedPoints.add(points.get(a));
    //         }
    //         else
    //         {
    //            int b=a;
    //            while (thetas[b] < thetas[b-1])
    //            {
    //               double temp = thetas[b-1];
    //               thetas[b-1] = thetas[b];
    //               thetas[b] = temp;
    //               b--;
    //               if (b==0)
    //                  break;
    //            }
    //            orderedPoints.add(b, points.get(a));
    //         }
    //            
    //      }
    //      return orderedPoints;
    //   }
    //  
    //   public void findVisibleSlice_3dRays(Point3d insidePoint) 
    //   {
    ////      // Choose only the points in the slice that are visible to insidePoint
    //////      mesh.checkFaceNormals();
    //////      mesh.faceNormalsValid();
    ////      mesh.updateFaceNormals();
    ////      //mesh.computedFaceNormals();
    ////      
    ////      // 1) find only the points that can see the "inside point"
    ////      ArrayList<Point3d> visiblePoints = new ArrayList<Point3d>();
    ////      for (Point3d point : slice.getWorldPoints())
    ////      {
    ////       Vector3d rayDirC2F = new Vector3d();
    ////       rayDirC2F.sub(point, insidePoint);                     // vector from centerline to face
    ////       //rayDirC2F.scale(0.75);
    ////       Vector3d rayDirF2C = rayDirC2F.clone();
    ////       rayDirF2C.scale(-1.0);
    ////       //rayDirC2F.normalize();
    ////       
    ////       Point3d t1 = nearestPointIntersectedByRay(mesh, insidePoint, rayDirC2F);
    ////       Point3d t2 = nearestPointIntersectedByRay(mesh, insidePoint, rayDirF2C);
    ////       Point3d intPoint = t1;         // I think it *should* be t1!
    ////       if (intPoint == null)
    ////       {
    ////          visiblePoints.add(point);
    ////       }
    ////       else if (point.distance(intPoint) < 0.0000001)
    ////       {
    ////          visiblePoints.add(point);
    ////       }
    //////     if (t1.distance(t2) > 0.000001)
    //////     {
    //////        // ray direction matters --> this should always be the case
    //////        int temp = 1;
    //////     }
    //////     else
    //////     {
    //////        // ray direction doesn't matter --> grrrr!
    //////        int temp = 1;
    //////     }
    //////     
    //////
    //////     Intersector intersector = new Intersector();           // what is this?
    //////     Vector3d duv = new Vector3d();                         // what is this? does it need to be new for each collision?
    //////
    //////     //Vector3d nearest = new Vector3d();
    //////     //Face faceHit = mesh.getBvHierarchy().nearestFaceToPoint(point, nearest, new Point3d(), new Vector2d(), new Intersector());
    //////     // TODO: this faceHit method below seems to be completely wrong.  Why?
    //////     Face faceHit = mesh.getBvHierarchy().nearestFaceIntersectedByRay(insidePoint, rayDirC2F, duv, intersector); // --> calc ray to face, then use this
    //////     if (faceHit != null)   // theoretically, this should never be null...
    //////     {
    //////        Point3d wCent = new Point3d();
    //////        faceHit.computeWorldCentroid(wCent);
    //////        Plane plane = new Plane(faceHit.getWorldNormal(), wCent);
    //////        Point3d iSect = new Point3d();
    //////        double dist = plane.intersectLine(iSect, rayDirC2F, insidePoint);
    //////        if (Math.abs(point.norm() - iSect.norm()) < 0.000001)
    //////        {
    //////           visiblePoints.add(point);
    //////        }
    //////     }
    //////     else
    //////     {
    //////        // these points *should* hit something; if they numerically miss, they are included
    //////        // TODO: is this a safe assumption always??
    //////        visiblePoints.add(point);
    //////     }
    ////      }
    ////      
    ////      slice.setWorldPoints(visiblePoints);
    ////      //slice.setPlane(sliceOrigin, sliceNormal);
    ////      slice.calculatePlanePoints();
    ////      slice.orderPointsByAngle(insidePoint);
    //   }
    //   
    //   public static Point3d nearestPointIntersectedByRay(PolygonalMesh mesh, Point3d rayOrigin, Vector3d rayDir)
    //   {
    //      return nearestPointIntersectedByRay(mesh, rayOrigin, rayDir, new Face(0), 0.0);
    //   }
    //   
    //   public static Point3d nearestPointIntersectedByRay(PolygonalMesh mesh, Point3d rayOrigin, Vector3d rayDir, Face intersectedFace, double dist)
    //   {
    //      
    //      //TriangleIntersector intersector = new TriangleIntersector();            // what is this?
    //      BVFeatureQuery query = new BVFeatureQuery();
    //      Vector3d duv = new Vector3d();                            // what is this? does it need to be new for each collision?
    //
    //      //intersectedFace = mesh.getBvHierarchy().nearestFaceIntersectedByRay(rayOrigin, rayDir, duv, intersector); // --> calc ray to face, then use this
    //      intersectedFace = query.nearestFaceAlongRay (
    //         null, duv, mesh, rayOrigin, rayDir);
    //      if (intersectedFace != null)
    //      {
    //         Point3d wCent = new Point3d();
    //         intersectedFace.computeWorldCentroid(wCent);
    //         Plane plane = new Plane(intersectedFace.getWorldNormal(), wCent);
    //         Point3d iSect = new Point3d();
    //         dist = plane.intersectLine(iSect, rayDir, rayOrigin);
    //         return iSect;
    //      }
    //      else
    //      {
    //         return null;
    //      }
    //   }

}
