package artisynth.models.fluid1d.fluidUtils;

import java.awt.Color;
import java.util.ArrayList;

import maspack.matrix.*;
import maspack.geometry.*;
import maspack.render.RenderProps;
import artisynth.core.gui.*;
import artisynth.core.driver.*;
import artisynth.core.femmodels.*;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;

public class FluidUtils
{
   //TODO: replace "area of face" calls with native method, group geom calculation code
   
   // --- calculation helpers --- //
   
   // Rotate from world coordinates to plane coordinates where x',y' are in the plane and z' is the plane normal
   public static RotationMatrix3d WorldToPlaneRotation(Vector3d planeNormal)
   {
      
      Vector3d c = new Vector3d(planeNormal);               // c = z' = plane normal
      Vector3d b = new Vector3d(0.0, 1.0, 0.0);
      Vector3d a = new Vector3d(1.0, 0.0, 0.0);
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
      
      //System.out.println("Rot: " + rot.toString ());
      
      return rot;
   }
   
   static Vector3d GramSchmidtHelper(Vector3d v1, Vector3d v2)
   {
      // returns v1'*v2/(v1'*v1)*v1
      Vector3d res = new Vector3d();
      res.scale ((v1.dot(v2))/(v1.dot(v1)), v1);
      return res;
   }
   
   static AffineTransform3d WorldToPlaneTransformation(Vector3d normal, Point3d origin)
   {
      AffineTransform3d at = new AffineTransform3d();
      at.setRotation(WorldToPlaneRotation(normal));
      at.setTranslation(origin);
      return at;
   }
   
   public ArrayList<Point3d> TransformPoints(ArrayList<Point3d> input, RotationMatrix3d rot)
   {
      ArrayList<Point3d> output = new ArrayList<Point3d>(input.size ());
      
      for (Point3d pw : input)
      {
         pw.transform (rot);
      }
      
      return output;
   }
   
   // find area of general polygon
   public static double AreaOfPolygon(ArrayList<Point3d> points)
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
   
   public static double PerimeterOfPolygon(ArrayList<Point3d> points)
   {
      int N = points.size();
      double perim = points.get(0).distance(points.get(N-1));
      for (int a=1; a<N; a++)
      {
	 perim = perim + points.get(a).distance(points.get(a-1));
      }
      return perim;
   }
   
   public static void CalculateCentroidOfPoints(ArrayList<Point3d> points, Point3d centroid)
   {
      centroid.setZero();
      for (Point3d p : points) 
      {
         centroid.add (p);
      }
      centroid.scale (1.0 / (double)points.size());
   }
   
   // find the area of a face (just triangle for now)
   public static double AreaOfFace (Face f)
   {
      // for a *parallelogram* with vertices A,B,C,D --> Area = |AB x AC|
      // for a triangle with vertices A,B,C          --> Area = 0.5*|AB x AC|
      double area;
      
      Vector3d vAB = new Vector3d();
      Vector3d vAC = new Vector3d();
      
      vAB.sub (f.getVertex(0).pnt, f.getVertex(1).pnt);
      vAC.sub (f.getVertex(0).pnt, f.getVertex(2).pnt);
      
      vAB.cross (vAC);
      area = vAB.norm ();
      
      if (f.numVertices() == 3)
      {
         area = 0.5*area;               // this should be exact for all triangles
      }
      else if (f.numVertices() == 4)
      {
         // OK, this case is a bit of a hack; the area is correct for a parallelogram
      }
      else
      {
         System.out.println ("Area of face error: high-order face");
      }
      
      return area;
   }
   
   // apply a force for a face (by distributing evenly over the nodes)
//   public static void ApplyForceOnFace(Face f, Vector3d force)
//   {
//      int numV = f.numVertices();
//      Vector3d forcePerNode = new Vector3d();
//      forcePerNode.scale (1.0/numV, force);
//      
//      for (int a=0; a<numV; a++)
//      {         
//         //FemNode3d node3d = fem.getSurfaceNode(f.getVertex(a));
//         FemNode3d node3d = (FemNode3d)( ((FemMeshVertex)f.getVertex(a)).getPoint() );
//         node3d.getExternalForce().add(forcePerNode);
//      }
//   }
//   
//   public static void ApplyPressureOnFace(Face f, double p)
//   {
//      // the pressure acts perpendicular to face (opposite of normal)
//      Vector3d pForce = new Vector3d();
//      pForce.scale( -1.0*p*AreaOfFace(f), f.getNormal() );                     // resolve the pressure to a force
//      
//      ApplyForceOnFace(f, pForce);                                             // apply the force
//   }
   
   public static void SetTransformation(TransformableGeometry model, Vector3d translation, AxisAngle rotation, double[] scaling)
   {
      AffineTransform3d trans = new AffineTransform3d();
      trans.set (new RigidTransform3d ( translation, rotation ));
      trans.applyScaling (scaling[0], scaling[1], scaling[2]);                  // should I scale first?
      model.transformGeometry (trans);
   }
   
   public static AxisAngle AxisAngleBetweenVectors(Vector3d v1, Vector3d v2)
   {
      // note: the AxisAngle is dependent on vector order: it describes the rotation from v1 to v2
      Vector3d axis = new Vector3d();
      axis.cross (v1, v2);
      
      if (axis.norm() == 0.0)
         return new AxisAngle();                                // no rotation
      else
      {
         double theta = Math.atan2 (axis.norm(), v1.dot(v2) );
         axis.normalize ();                                     // I'm not sure if axis must be a unit vector here or not...
         
         return new AxisAngle(axis, theta);
      }
   }
   
   // --- some visualization helpers --- //
      
   public static void CreateAxes(MechModel mechmod, double scale)
   {
      // Create an Origin marker //
      RigidBody origin = new RigidBody();
      origin.setName ("origin");
      origin.setMesh (MeshFactory.createSphere (scale/25.0, 10), null);
      origin.setDynamic (false);
      origin.setPose (new RigidTransform3d ( new Vector3d (0, 0, 0), new AxisAngle (0, 0, 0, Math.PI*0.0)));
      
      // define the render properties
      RenderProps.setFaceColor (origin, Color.white);
      RenderProps.setAlpha (origin, 1.0);
      
      // add the object
      mechmod.addRigidBody(origin);
      // create the axes
      mechmod.addRigidBody( CreateAxis( new RigidTransform3d ( new Vector3d (scale/2.0, 0, 0), new AxisAngle ( 0, 1, 0, Math.PI*0.5)), scale, "xAxis", Color.red));
      mechmod.addRigidBody( CreateAxis( new RigidTransform3d ( new Vector3d (0, scale/2.0, 0), new AxisAngle (-1, 0, 0, Math.PI*0.5)), scale, "yAxis", Color.green));
      mechmod.addRigidBody( CreateAxis( new RigidTransform3d ( new Vector3d (0, 0, scale/2.0), new AxisAngle ( 0, 0, 0, Math.PI*0.0)), scale, "zAxis", Color.blue));
   }
   
   static RigidBody CreateAxis(RigidTransform3d pose, double scale, String name, Color color)
   {
      double l = scale;
      double r = l/40.0;
      double lTip = l/10.0;
      
      RigidBody axis = new RigidBody();
      axis.setName (name);
      axis.setMesh (MeshFactory.createPointedCylinder (r, l, lTip, 4), null);
      axis.setDynamic (false);
      axis.setPose (pose);
      
      // define the render properties
      RenderProps.setFaceColor (axis, color);
      RenderProps.setAlpha (axis, 1.0);
      
      // add the object
      //mechmod.addRigidBody(axis);
      return axis;
   }
   
   public static RigidBody CreateMeasurementPlane(RigidTransform3d pose, double[] dim,  Color color)
   {
      RigidBody plane = new RigidBody();
      plane.setName ("plane");
      plane.setMesh (
         MeshFactory.createRectangle (dim[0], dim[1], /*textureCoords=*/true), null);
      plane.setDynamic (false);
      plane.setPose (pose);
      
      // define the render properties
      RenderProps.setFaceColor (plane, color);
      RenderProps.setAlpha (plane, 0.5);
      
      // add the object
      //mechmod.addRigidBody(plane);
      return plane;
   }
   
//   public static ArrayList<FemElement> FindElementFromFace(Face face)
//   {
//      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
//      ArrayList<FemElement> fems = new ArrayList<FemElement>();
//      
//      for (int a=0; a<3; a++)
//      {
//	 //face.getVertex(a);
//	 FemNode3d node3d = (FemNode3d)( ((FemMeshVertex)face.getVertex(a)).getPoint() );
//	 nodes.add(node3d);
//	 //node3d.getElementDependencies().
//	 //fems.addAll(node3d.getElementDependencies());
//	 //node3d.getElementDependencies().get(0).
//      }
//      for (FemNode3d node : nodes)
//      {
//	 for (FemElement elem :node.getElementDependencies())
//	 {
//	    if ( (elem.containsNode(nodes.get(0))==true) && (elem.containsNode(nodes.get(1))==true) && (elem.containsNode(nodes.get(2))==true) )
//	    {
//	       fems.add(elem);
//	    }
//	 }
//      }
//      
//      return fems;
//   }
   
   public static ArrayList<Point3d> DoubleArrayToPoint3d(double[][] points)
   {
      // assuming nPoints x 3
      int nPoints = points.length;
      ArrayList<Point3d> pOut = new ArrayList<Point3d>(nPoints);
      for (int a=0; a<nPoints; a++)
      {
	 Point3d p = new Point3d(points[a]);
	 pOut.add(p);
      }
      return pOut;
   }
   
   public static double[][] Point3dToDoubleArray(ArrayList<Point3d> points)
   {
      int nPoints = points.size();
      double[][] data = new double[nPoints][3];
      for (int a=0; a<nPoints; a++)
      {
	 data[a][0] = points.get(a).x;
	 data[a][1] = points.get(a).y;
	 data[a][2] = points.get(a).z;
      }
      return data;
   }
   
   public static double[] GetModelBounds(PolygonalMesh mesh)
   {
      Vector3d min = new Vector3d();
      Vector3d max = new Vector3d();
      mesh.getWorldBounds(min, max);
      
      double[] bounds = {min.x, max.x, min.y, max.y, min.z, max.z};
      return bounds;
      
   }
   
   public static void PrintModelBounds(PolygonalMesh mesh)
   {
      double[] b = GetModelBounds(mesh);
      System.out.println( String.format("xMin=%f, xMax=%f, yMin=%f, yMax=%f, zMin=%f, zMax=%f", b[0], b[1], b[2], b[3], b[4], b[5]) );
   }
   
   public static double[] deepCopy_doubleArray(double[] arr)
   {
      double[] copy = new double[arr.length];
      for (int a=0; a< arr.length; a++)
	 copy[a] = arr[a];
      
      return copy;
   }
   
   public static Point3d ClosestPointOnLineSegment(Point3d linePt1, Point3d linePt2, Point3d point)
   {
      // line segment defined by linePt1 and linePt2, and point is where we are measuring from
      Point3d linePoint;
      double[] a = {linePt1.x, linePt1.y, linePt1.z};
      double[] b = {linePt2.x, linePt2.y, linePt2.z};
      double[] p = {point.x, point.y, point.z};
      
      // r = (ap.ab)/||ab||^2
      double[] ap = ArrayMath.sub(p,a);
      double[] ab = ArrayMath.sub(b,a);
      double AB = ArrayMath.norm(ab);
      double r = ArrayMath.dot(ap,ab)/(AB*AB);
      
      if (r<0.0)
	 linePoint = linePt1;
      else if (r>1.0)
	 linePoint = linePt2;
      else
      {
	 double[] pLine = ArrayMath.add(a, ArrayMath.mult(ab, r));	// this is the point on the line
	 linePoint = new Point3d(pLine);
      }
	 
      return linePoint;
   }
   
   public static double DistanceToLineSegment(Point3d linePt1, Point3d linePt2, Point3d point)
   {
      return point.distance( ClosestPointOnLineSegment(linePt1, linePt2, point) );
   }
   
//   public static Point3d findEdgePlaneIntersection(Plane plane, HalfEdge edge)
//   {
//      // TODO: this code is completely untested!
//      Point3d iSect = new Point3d();
//      Vector3d dir = new Vector3d();
//      edge.getNormalizedDirection(dir);
//      Point3d pnt = edge.getTail().getPosition();
//      double dist = plane.intersectLine(iSect, dir, pnt);
//      if ( (dist < 0) || (dist > edge.length()) )
//      {
//	 return null;
//      }
//      else
//      {
//	 return iSect;
//      }
//   }
   
   Point3d RayPlaneIntersection(Point3d planeOrigin, Vector3d planeNormal, Point3d rayOrigin, Vector3d rayDirection)
   {
      // I'm not sure how robust this is...
      Point3d point = new Point3d();
      Plane plane = new Plane(planeNormal, planeOrigin);
      double dist = plane.intersectLine(point, rayDirection, rayOrigin);
      
      return point;
   }

   public static double Interpolate1D_linear(double[] x, double[] f_x, double xVal)
   {
      if (x.length != f_x.length)
         return Double.NaN;
      
      int N = x.length;
      int i = 0;
      
      while ((Math.abs(x[i]-xVal) > Math.abs(x[i+1]-xVal)) && (i<N-2))
         i++;
      
      double f = ( (f_x[i] - f_x[i+1])/(x[i] - x[i+1]) )*(xVal-x[i]) + f_x[i];
      
      return f;
   }

}
