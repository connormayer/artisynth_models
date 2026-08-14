package artisynth.models.fluid1d.csa;

import java.util.ArrayList;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Marker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.RigidBody;

/*//
 * Author: petera
//*/

public class SliceInterpreter_Components extends SliceInterpreter
{

    //ArrayList<Point3d> points;
    ArrayList<Marker> markers;

    MechModel mechModel;
    ArrayList<FemModel3d> fems;
    ArrayList<RigidBody> rbs;
    ArrayList<PolygonalMesh> geometries;

    // constructors (same as super class)
    public SliceInterpreter_Components()
    {
        super();
    }

    public SliceInterpreter_Components(String name, Point3d planeCenter, Vector3d planeNormal)
    {
        super(name, planeCenter, planeNormal);
    }

    public SliceInterpreter_Components(String name, ArrayList<Point3d> points)
    {
        super(name, points);
    }

    // implementations
    public void initialize()
    {
        areaMesh = new PolylineMesh();
        if (findSlice == true)
        {
            geometries = new ArrayList<PolygonalMesh>();
            for (RigidBody rb : rbs)
                geometries.add(rb.getMesh());
            for (FemModel3d fem : fems)
                geometries.add(fem.getSurfaceMesh());
            gs.setGeometries(geometries);
        }
        else
        {
            attachPointsAsMarkers();
        }
        update();
    }

    //   public void update()
    //   {
    //      // many options from the area calculator could be added here if needed...
    //      if (findSlice == true)
    //      {
    //	 gs.update();				// update the slice of the entire geometry
    //
    //	 if (gs.slice.numLines() > 0)
    //	 {
    //	    //RigidTransform3d transform = AreaCalculator.calcWorldToPlaneTransformation(sliceNormal, sliceOrigin);
    //	    Point3d pointXY = new Point3d( gs.getPlaneOrigin() );      
    //	    pointXY.inverseTransform( gs.slice.getMeshToWorld() );
    //	         
    //	    areaMesh = findVisibleSlice_2dExact(gs.slice, pointXY);
    //	    points = new ArrayList<Point3d>(areaMesh.numVertices());
    //	    for (int a=0; a<areaMesh.numVertices(); a++)
    //	       points.add(areaMesh.getVertices().get(a).getWorldPoint());
    //	    area = AreaCalculator.calcArea3d(points);	       
    //	 }
    //	 else
    //	 {
    //	    areaMesh = new PolylineMesh();
    //	    area = Double.NaN;
    //	 }
    //	 
    //	 if (sliceEachStep == false)
    //	 {
    //	    findSlice = false;
    //	    attachPointsAsMarkers();
    //	 }
    //      }
    //      else
    //      {
    //	 area = AreaCalculator.calcArea3d(points);	// TODO: are points safe, or should it be markers??
    //      }
    //   }

    public void attachPointsAsMarkers()
    {
        //ArrayList<Point3d> points = new ArrayList<Point3d>(areaMesh.numVertices()); 
        markers = new ArrayList<Marker>(areaMesh.numVertices());
        PolylineMesh newMesh = new PolylineMesh();

        for (Polyline pl : areaMesh.getLines())
        {
            int nPoints = pl.numVertices();
            Vertex3d[] verts = new Vertex3d[nPoints+1];

            for (int a=0; a<nPoints; a++)
            {
                //Point3d p = v.pnt;
                Point3d p = pl.getVertex(a).getWorldPoint();
                Marker m = attachPointToNearestModel(p);
                Vertex3d v = newMesh.addVertex(m.getPosition(), true);
                verts[a] = v;
                //points.add(m.getPosition());
                markers.add(m);
            }
            verts[nPoints] = verts[0];
            newMesh.addLine(verts);

        }
        // TODO: could it be a problem that the new areaMesh no longer uses local cutplane coordinates?
        //newMesh.setMeshToWorld(areaMesh.getMeshToWorld());
        areaMesh = newMesh;
    }

    public Marker attachPointToNearestModel(Point3d point)
    {
        Marker marker;

        // find the nearest model and attach point as a marker to that model
        RigidBody nearestRB = null;
        FemModel3d nearestFEM = null;
        double minDist = Double.POSITIVE_INFINITY;

        for (RigidBody rb: rbs)		// find nearest rb
        {
            Point3d nearest = new Point3d();
            BVFeatureQuery.getNearestFaceToPoint (
                nearest, null, rb.getMesh(), point);
            //rb.getMesh().getObbtree().nearestFace(point, new Vector3d(), nearest, new Vector2d(), new TriangleIntersector());
            double dist = nearest.distance(point);
            if (dist < minDist)
            {
                nearestRB = rb;
                minDist = dist;
            }
        }
        for (FemModel3d fem : fems)	// find nearest fem
        {
            Point3d nearest = new Point3d();
            BVFeatureQuery.getNearestFaceToPoint (
                nearest, null, fem.getSurfaceMesh(), point);
            //fem.getSurfaceMesh().getObbtree().nearestFace(point, new Vector3d(), nearest, new Vector2d(), new TriangleIntersector());
            double dist = nearest.distance(point);
            if (dist < minDist)
            {
                nearestRB = null;
                nearestFEM = fem;
                minDist = dist;
            }
        }

        if (nearestRB != null)
        {
            FrameMarker m = new FrameMarker(nearestRB, point);
            mechModel.addFrameMarker(m);
            marker = m;
        }
        else if (nearestFEM != null)
        {
            FemMarker m = new FemMarker(point);
            nearestFEM.addMarker(m);
            marker = m;
        }
        else
        {
            marker = null;
        }

        return marker;
    }


    public void setModels(MechModel mm, ArrayList<FemModel3d> fems, ArrayList<RigidBody> rbs)
    {
        if (fems == null)		// then add all FEMs...	
        {
            fems = new ArrayList<FemModel3d>();
            for (MechSystemModel msm: mm.models())
                if (msm instanceof FemModel3d)
                    fems.add((FemModel3d)msm);
        }
        if (rbs == null)		// then add all RBs...
        {
            rbs = new ArrayList<RigidBody>();
            for (RigidBody rb : mm.rigidBodies())
                rbs.add(rb);
        }

        this.mechModel = mm;
        this.fems = fems;
        this.rbs = rbs;
    }

}
