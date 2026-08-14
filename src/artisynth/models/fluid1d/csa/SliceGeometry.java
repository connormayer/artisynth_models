package artisynth.models.fluid1d.csa;

import java.util.ArrayList;


import maspack.collision.ContactInfo;
import maspack.collision.ContactPlane;
import maspack.collision.IntersectionContour;
import maspack.collision.MeshCollider;
import maspack.collision.IntersectionPoint;
import maspack.collision.SurfaceMeshCollider;
import maspack.geometry.*;
import maspack.matrix.*;

/*//
 * Author: petera
 * This class takes input geometries and a cutplane, and finds the slice (points and connectivity).
 * This is done using the MeshCollider, and deriving the points/connectivity from the intersection
 * of a geometry and cutplane.
//*/

public class SliceGeometry
{
    // the cut plane
    Point3d sliceOrigin;
    Vector3d sliceNormal;
    PolygonalMesh cutplane;

    ArrayList<PolygonalMesh> geoms;      // input geometry(s)

    // output slice: the mesh coords of the slice have the x-y plane in the coincident with the cutplane
    PolylineMesh slice;

    public SliceGeometry()
    {
    }

    public SliceGeometry(ArrayList<PolygonalMesh> geometries, Vector3d planeNormal, Point3d planeOrigin)
    {
        setGeometries(geometries);
        setPlane(planeOrigin, planeNormal);
    }

    public void update()
    {  
        // 1. care must be taken to define a cutplane that extends beyond all the input geometries
        Point3d pMin = new Point3d();
        Point3d pMax = new Point3d();

        slice = new PolylineMesh();
        for (PolygonalMesh geometry : geoms)
        {
            geometry.updateBounds(pMin, pMax);					// find the bounds of geometry in world coordinates
            cutplane = createCutplane(sliceNormal, sliceOrigin, pMin, pMax);	// create cutplane extending beyond geometry
            //PolylineMesh slicePart = calculateSlice(geometry, cutplane);		// calculate the intersection of cutplane and geometry
            PolylineMesh slicePart = calculateSlice_ixer(geometry, cutplane);          // calculate the intersection of cutplane and geometry
            slice.addMesh(slicePart);						// add intersection to the global section
            //addGeometryToBase(slice, slicePart);
        }

        slice.inverseTransform(cutplane.getMeshToWorld());	// slice is calculated in world coords; trans to mesh coords
        slice.setMeshToWorld(cutplane.getMeshToWorld());		// but make sure the slice has world coords defined 

    }

    public static PolylineMesh calculateSlice_ixer(PolygonalMesh geometry, PolygonalMesh cutplane)
    {
        // use the SurfaceMeshCollider
        PolylineMesh contours = new PolylineMesh();
        SurfaceMeshCollider myIxer = new SurfaceMeshCollider ();
        ArrayList<IntersectionContour> contourList = 
           myIxer.getContours(geometry, cutplane);

        if (contourList != null)
        {
            int nContours = contourList.size();
            for (int n=0; n<nContours; n++)
            {
                //myIxer.getContours().get(n)
                IntersectionContour points = contourList.get(n);
                boolean isOpen = !points.isClosed();
                int nPoints = points.size();

                Vertex3d[] verts;
                if (isOpen == true)
                    verts = new Vertex3d[nPoints];
                else
                    verts = new Vertex3d[nPoints+1];

                for (int i=0; i<nPoints; i++)
                {
                    Vertex3d vert =new Vertex3d(points.get(i));
                    contours.addVertex(vert);
                    verts[i] = vert;
                }
                if (isOpen == false)
                    verts[nPoints] = verts[0];     // close the contour

                contours.addLine(verts);
            }
        }  

        return contours;
    }

    public static PolylineMesh calculateSlice(PolygonalMesh geometry, PolygonalMesh cutplane)
    {
        /*/
         * Derive contours based on a list of TriTriIntersections. To do this:
         * 1) remove all TTIs that have a points at the same place (hence line segments of length 0)
         * 2) remove all TTIs that have line segments that are the same as another TTI
         * 3) create a contour list (that is, ordered points) by connecting TTIs with line segments that share an endpoint
         * This code will probably fail if the TTI contains more or less than 2 points (for ex, there may be 3 points for coincident planes)
      //*/

        PolylineMesh contours = new PolylineMesh();

        double tol = 1e-10;

        MeshCollider collider = new MeshCollider();
        ContactInfo info = collider.getContacts(geometry, cutplane);        // this finds the intersection points, but not connectivity
        if (info == null)
            return contours;

        ArrayList<ContactPlane> regions = info.getContactPlanes();
        int nConts = regions.size();
        for (int n=0; n<nConts; n++)
        {
            ArrayList<TriTriIntersection> isects = regions.get(n).intersections;
            int I = isects.size();         // max index
            if (I == 0)
                break;
            ArrayList<Point3d> contour = new ArrayList<Point3d>();

            // remove segments of length 0 from isects
            ArrayList<TriTriIntersection> ttTemp = new ArrayList<TriTriIntersection>();
            for (TriTriIntersection tt : isects)
                if (tt.points[0].distance(tt.points[1]) < tol/10.0)
                    ttTemp.add(tt);
            isects.removeAll(ttTemp);
            I = isects.size();
            //System.out.printf("Zero-length reduction: nSets = %d, %d removed.\n", I, ttTemp.size());

            // remove repetitive segments from isects (both points are the same)
            ttTemp = new ArrayList<TriTriIntersection>();
            for (int a=0; a<isects.size()-1; a++)
            {
                Point3d p0a = isects.get(a).points[0];
                Point3d p0b = isects.get(a).points[1];
                for (int b=a+1; b<isects.size(); b++)
                {
                    if ( (p0a.distance(isects.get(b).points[0]) < tol) && (p0b.distance(isects.get(b).points[1]) < tol) )
                    {
                        ttTemp.add(isects.get(a));
                        break;
                    }
                    else if ( (p0a.distance(isects.get(b).points[1]) < tol) && (p0b.distance(isects.get(b).points[0]) < tol) )
                    {
                        ttTemp.add(isects.get(a));
                        break;
                    }
                }
            }
            isects.removeAll(ttTemp);
            I = isects.size();
            //System.out.printf("Same-set reduction: nSets = %d, %d removed.\n", I, ttTemp.size());

            // now we form the contour by connecting segments that have endpoints within tol of each other
            contour.add(isects.get(0).points[0]);
            contour.add(isects.get(0).points[1]);
            isects.remove(0);
            Point3d p_init = contour.get(0);
            Point3d p_curr = contour.get(1);

            boolean contour_is_open = false;       // A break condition if contour isn't closed
            while (p_init.distance(p_curr) > tol)
            {
                boolean pointFound = false;
                ttLoop:
                    for (TriTriIntersection tt : isects)
                    {
                        //for (Point3d p : tt.points)
                        for (int i=0; i<tt.points.length; i++)
                        {
                            Point3d p = tt.points[i];
                            if (p_curr.distance(p) < tol)
                            {
                                // the next "current" point is the non-matching one. This assumes just 2 points in tt
                                if (i==0)
                                    p_curr = tt.points[1];
                                else
                                    p_curr = tt.points[0];
                                contour.add(p_curr);
                                isects.remove(tt);
                                pointFound = true;

                                break ttLoop;     //
                            }
                        }
                    }
                if (pointFound == false)
                {
                    if (contour_is_open == false)
                    {
                        // we reached one end of the contour, return to the start point and traverse the other direction
                        int nP = contour.size();
                        ArrayList<Point3d> temp = contour;
                        contour = new ArrayList<Point3d>(nP);

                        for (int a=nP-1; a>=0; a--)
                            contour.add(temp.get(a));
                        p_curr = contour.get(nP-1);
                        p_init = contour.get(0);

                        contour_is_open = true;
                    }
                    else
                    {
                        // we've already found one edge of the open contour, and now we've reached the other. So we are done.
                        break;
                    }
                }
            }
            // Finally, add the ordered points as a contour in the polyline mesh
            int nVerts = contour.size();
            Vertex3d[] vertsArr = new Vertex3d[nVerts];
            for (int a=0; a<nVerts; a++)
            {
                Vertex3d v = contours.addVertex( contour.get(a) );
                vertsArr[a] = v;
            }
            contours.addLine(vertsArr);

            //System.out.printf("Contour found with %d of %d points, open = %b \n", nVerts, I, contour_is_open);

        }
        return contours;
    }

    // if only the (unordered) points are needed, use this method...
    public static PolylineMesh calculateSlicePoints(PolygonalMesh geometry, PolygonalMesh cutplane)
    {
        PolylineMesh slice = new PolylineMesh();

        MeshCollider collider = new MeshCollider();                             // the default collider??
        //SurfaceMeshCollider collider = new SurfaceMeshCollider();                 // this is the AJL collider (uses SurfaceMeshIntersector)
        ContactInfo info = collider.getContacts(geometry, cutplane);
        ArrayList<ContactPlane> regions = info.getContactPlanes();
        int nConts = regions.size();

        for (int n=0; n<nConts; n++)
        {

            ArrayList<Point3d> points = regions.get(n).points; 
            int nPoints = points.size();
            Vertex3d[] verts = new Vertex3d[nPoints+1];

            for (int i=0; i<nPoints; i++)
            {
                Vertex3d vert =new Vertex3d(points.get(i));
                slice.addVertex (vert);
                verts[i] = vert;
            }
            verts[nPoints] = verts[0];     // close the contour

            slice.addLine(verts);
        }

        return slice;

        /*//
      // If SurfaceMeshIntersector worked consistently, this would be the preferred way.
      SurfaceMeshIntersector smi = new SurfaceMeshIntersector();
      smi.findContours(geometry, cutplane);
      for (MeshIntersectionContour mic : smi.contours)
      {

         int nPoints = mic.size();
         Vertex3d[] verts;
         if (mic.isClosed == true)
            verts = new Vertex3d[nPoints+1];
         else
            verts = new Vertex3d[nPoints];

         for (int i=0; i<nPoints; i++)
         {
            Vertex3d vert =new Vertex3d(mic.get(i));
            slice.addVertex (vert);
            verts[i] = vert;
         }
         if (mic.isClosed == true)
            verts[nPoints] = verts[0];
         slice.addLine(verts);
      }
      //*/
    }

    public static PolygonalMesh createCutplane(Vector3d sliceNormal, Point3d sliceOrigin, Point3d bbMin, Point3d bbMax)
    {
        // bbMin, bbMax describe the bounding box which all geometric edges of the cutplane must lie outside of (mesh edges may be inside)
        // this approach could be nicer...but works for now
        RigidTransform3d transform = AreaCalculator.calcWorldToPlaneTransformation(sliceNormal, sliceOrigin);
        transform.invert();	// xPlaneToMesh

        double scale = 1.2;
        
//        double dx = bbMax.x - bbMin.x;
//        double dy = bbMax.y - bbMin.y;
//        double dz = bbMax.z - bbMin.z;
//        double maxDim = Math.sqrt(dx*dx + dy*dy + dz*dz)*scale;
        
        double maxDim = (sliceOrigin.distance(bbMax) + sliceOrigin.distance(bbMin)) * 2.0 * scale; // should ensure a wide enough cutplane
        
        PolygonalMesh cutplane = 
           MeshFactory.createRectangle(maxDim, maxDim, /*textureCoords=*/true);
        cutplane.setMeshToWorld(transform);
        //cutplane.inverseTransform(transform);

        return cutplane;
    }

    public static PolygonalMesh createCutplane_notWorking(Vector3d sliceNormal, Point3d sliceOrigin, Point3d pMin, Point3d pMax)
    {
        // would create a better fitting cutplane...if it worked...
        // 1. care must be taken to define a cutplane that extends beyond all the input geometries
        RigidTransform3d transform = AreaCalculator.calcWorldToPlaneTransformation(sliceNormal, sliceOrigin);

        // find the bounds of all the models in world coordinates
        double dx = pMax.x - pMin.x;
        double dy = pMax.y - pMin.y;
        double dz = pMax.z - pMin.z;

        // create a bounding box for all objects, and then rotate it to the cutplane coordinates
        double scale = 1.2;
        PolygonalMesh boundingBox = MeshFactory.createQuadBox(dx*scale, dy*scale, dz*scale, pMin.x+dx/2.0, pMin.y+dy/2.0, pMin.z+dz/2.0);
        boundingBox.transform(transform);		// rotate the bounding box to cutplane coords (plane lies in x,y)
        Point3d bbMin = new Point3d();
        Point3d bbMax = new Point3d();
        boundingBox.getWorldBounds(bbMin, bbMax);

        // hack for now...
        //      RigidBody bb = new RigidBody();
        //      bb.setName("boundingBox");
        //      bb.setMesh(boundingBox, null);
        //      boundingBox.inverseTransform(transform);
        //      bb.getRenderProps().setAlpha(0.25);
        //      bb.getRenderProps().setFaceStyle(maspack.render.Renderer.FaceStyle.FRONT_AND_BACK);
        //      bb.getRenderProps().setFaceColor(java.awt.Color.cyan);
        //      mechModel.addRigidBody(bb);

        // now, the x,y bounds of the bb will ensure that the cutplane extends beyond all geoms to be sliced  
        Point3d v1 = new Point3d(bbMin.x, bbMin.y, 0.0);
        Point3d v2 = new Point3d(bbMax.x, bbMin.y, 0.0);
        Point3d v3 = new Point3d(bbMin.x, bbMax.y, 0.0);
        Point3d v4 = new Point3d(bbMax.x, bbMax.y, 0.0);

        // define a cutplane that is larger than the mesh 
        PolygonalMesh cutplane = new PolygonalMesh();
        cutplane.addVertex(v1);
        cutplane.addVertex(v2);
        cutplane.addVertex(v3);
        cutplane.addVertex(v4);
        cutplane.addFace(new int[]{0,3,2});
        cutplane.addFace(new int[]{0,1,3});
        cutplane.inverseTransform(transform);		// and bring the cutplane back to world coords

        return cutplane;
    }

    // ----- Getters and Setters ----- //

    public void setGeometries(ArrayList<PolygonalMesh> geometries)
    {
        this.geoms = geometries;
    }

    public void setGeometry(PolygonalMesh geometry)
    {
        this.geoms = new ArrayList<PolygonalMesh>(1);
        geoms.add(geometry);
    }

    public void setPlane (Point3d planeOrigin, Vector3d planeNormal) 
    {
        this.sliceOrigin = planeOrigin;
        this.sliceNormal = planeNormal;
        this.sliceNormal.normalize ();
    }

    public Vector3d getPlaneNormal () 
    {
        return sliceNormal;
    }

    public Point3d getPlaneOrigin()
    {
        return sliceOrigin;
    }

    public PolylineMesh getSlice ()
    {
        return slice;
    }
    
    public PolygonalMesh getCutplane()
    {
        return cutplane;
    }

    // --- old stuff --- //
    public static void addGeometryToBase(PolygonalMesh baseGeom, PolygonalMesh addedGeom)
    {
        // first import the vertices
        Vertex3d[] newVertices = new Vertex3d[addedGeom.numVertices()];
        for (int a=0; a<addedGeom.numVertices(); a++)
        {
            Vertex3d vNew = new Vertex3d(addedGeom.getVertices().get(a).getWorldPoint());
            newVertices[a] = vNew;
            baseGeom.addVertex(vNew);
        }

        // build the faces
        for (Face f : addedGeom.getFaces())
        {
            Vertex3d[] fv = new Vertex3d[f.getVertexIndices().length];

            int index = 0;
            for (int b=0; b<fv.length; b++)
            {
                index = addedGeom.getVertices().indexOf(f.getVertex(b));	// index of the old vertex
                fv[b] = newVertices[index];					// add the new vertex
            }
            baseGeom.addFace(fv);
        }

        //geometry.updateFaceNormals();
    }

}
