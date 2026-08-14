package artisynth.models.fluid1d;

import java.util.ArrayList;

import artisynth.models.fluid1d.fileIO.VTK_IO;
import maspack.geometry.*;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

public class Coupler_3dTo1d implements Coupler 
{  
    GeometryCoupler_3DTo1D geometryCoupler = new GeometryCoupler_3DTo1D();
    //GeometryCoupler_3DTo1D_Ray geometryCoupler = new GeometryCoupler_3DTo1D_Ray();	// geometry coupling
    //GeometryCoupler_3DTo1D_Intersector geometryCoupler = new GeometryCoupler_3DTo1D_Intersector();	// geometry coupling
    //GeometryCoupler_3DTo1D_VTK geometryCoupler = new GeometryCoupler_3DTo1D_VTK();	// geometry coupling
    //ForceCoupler_1DTo3D forceCoupler = new ForceCoupler_1DTo3D_RBF();			// force coupling
    ForceCoupler_1DTo3D forceCoupler = new ForceCoupler_1DTo3D_Linear();			// force coupling

    StructureSolution structureSolution;							// structure solution
    FluidSolution_1D fluidSolution;							// fluid solution

    ArrayList<Face> fsiStructureFaces;		// should this be a geometry_3d?
    //ArrayList<Face> fsiFluidFaces;		// all fluid nodes (faces?) are fsi nodes

    //ArrayList<ArrayList<Point3d>> intersectionPoints;

    public void initialize()
    {
        // build up the geometry and force couplers here
        geometryCoupler.setInputSolution(structureSolution);
        geometryCoupler.setOutputSolution(fluidSolution);
        geometryCoupler.initialize();
        //intersectionPoints = geometryCoupler.getInterpolationPoints();

        forceCoupler.setFluidSolution(fluidSolution);
        forceCoupler.setStructureSolution(structureSolution);
        forceCoupler.setTargetFaces(fsiStructureFaces);
        if (forceCoupler.getClass().equals(ForceCoupler_1DTo3D_RBF.class))
        {
            ArrayList<ArrayList<Point3d>> interpPoints = new ArrayList<ArrayList<Point3d>>();
            for (PolylineMesh slice : geometryCoupler.getSlices())
            {
                ArrayList<Point3d> ipSlice = new ArrayList<Point3d>();
                for (Vertex3d v : slice.getVertices())
                {
                    ipSlice.add(v.getPosition());
                }
                interpPoints.add(ipSlice);
            }
            ((ForceCoupler_1DTo3D_RBF)forceCoupler).setInterpolationPoints(interpPoints);
        }
        forceCoupler.initialize();
    }

    // Find/Update FSI interface --> seemingly the faces might be moving in and out of the fsi interface in which case this would be needed

    public void updateGeometry()
    {
        //geometryCoupler.SetInterpolationPoints(MarkersToPoints(intersectionMarkers));		// is this update needed?
        geometryCoupler.update();
        fluidSolution.getGeometry().setArea(geometryCoupler.getOutputGeometry().getArea());
    }

    public void updateForces()
    {
        //forceCoupler.SetInterpolationPoints(MarkersToPoints(intersectionMarkers));
        forceCoupler.update();
    }

    public void setFluidSolution(FluidSolution_1D fs)
    {
        fluidSolution = fs;
    }

    public void setStructureSolution(StructureSolution ss)
    {
        structureSolution = ss;
    }

    public void setFSIStructureFaces(ArrayList<Face> fsiFaces)
    {
        fsiStructureFaces = fsiFaces;
    }

    public ArrayList<Face> getFSIStructureFaces()
    {
        return fsiStructureFaces;
    }

    public GeometryCoupler_3DTo1D getGeometryCoupler()
    {
        return this.geometryCoupler;
    }

    public ForceCoupler_1DTo3D getForceCoupler()
    {
        return this.forceCoupler;
    }

    public void findFSIInterface(PolygonalMesh fsiSurface, double tol)
    {
        // calculates the FSI interface based on fsiSurface, which may be either identical or an approximation of the fsi interface.

        PolygonalMesh solidGeom = structureSolution.getGeometry();
        ArrayList<Face> fsiFaces = new ArrayList<Face>();

        //OBBTree obbt = fsiSurface.getObbtree();
        //      BVTree obbt = fsiSurface.getBVTree();
        //      Point3d proj = new Point3d();
        //      Vector2d coords = new Vector2d();
        //Intersector isect = new Intersector();
        Point3d centroid = new Point3d();
        Point3d centroid_nf = new Point3d();
        double dist = 0.0;

        for (Face face : solidGeom.getFaces())
        {
            face.computeWorldCentroid(centroid);
            Face nearestFace = BVFeatureQuery.getNearestFaceToPoint(centroid_nf, null, fsiSurface, centroid);
            //Face nearestFace = obbt.nearestFace(centroid, null, proj, coords, isect);
            //nearestFace.nearestPoint(centroid_nf, centroid);	// does this work in world coords or local coords? Seems so.  Much better, though slower.
            //nearestFace.computeWorldCentroid(centroid_nf);	// ... than this line
            // the best would be comparing nearest point to nearest point...
            dist = centroid.distance(centroid_nf); 
            if ( (dist < tol) && (fsiFaces.contains(nearestFace) == false) )
                fsiFaces.add(face);
        }

        fsiStructureFaces = fsiFaces;
    }

    /*// Caution: unless fsisuface is perfect match (or much finer than solidGeom), this won't find all fsi faces
   public void findFSIInterface(PolygonalMesh fsiSurface)
   {
      // calculates the FSI interface based on fsiSurface, which may be either identical or an approximation of the fsi interface.

      PolygonalMesh solidGeom = structureSolution.getGeometry();
      ArrayList<Face> fsiFaces = new ArrayList<Face>();

      OBBTree obbt = solidGeom.getObbtree();
      Point3d proj = new Point3d();
      Vector2d coords = new Vector2d();
      Intersector isect = new Intersector();
      Point3d centroid = new Point3d();

      for (Face face : fsiSurface.getFaces())
      {
	 face.computeWorldCentroid(centroid);
	 Face nearestFace = obbt.nearestFace(centroid, null, proj, coords, isect);
	 if (fsiFaces.contains(nearestFace) == false)
	    fsiFaces.add(nearestFace);
      }

      fsiStructureFaces = fsiFaces;
   }
   //*/

    public void findFSIInterface()
    {
        // Performs a brute-force test to find which faces are visible to the centerline

        // some options
        double thetaMin = -180.0*Math.PI/180.0;		// angle between face normal and face-centerline vector must be greater than thetaMin to be included
        boolean onlyNearest = false;			// face must see the nearest point on centerline to be included

        ArrayList<Face> fsiFaces = new ArrayList<Face>(); // estimate N faces?

        PolygonalMesh geom = structureSolution.getGeometry();
        Centerline cl = fluidSolution.getGeometry().getCenterline();
        int N = cl.getNumberOfPoints();

        double[] dists = new double[N];
        double[] theta = new double[N];
        boolean[] vis = new boolean[N];
        Point3d clp  = new Point3d();
        Point3d fCent = new Point3d();
        Vector3d fNorm = new Vector3d();
        Vector3d rayDirC2F = new Vector3d();
        Vector3d rayDirF2C = new Vector3d();
        Face faceHit;
        int minLoc = 0;

        for (Face face : geom.getFaces())
        {
            for (int n=0; n<N; n++)
            {
                clp = cl.getVertices().get(n);		// my centerline is in world coordinates
                face.computeWorldCentroid(fCent);		// find face centroid in world coordinates
                fNorm = face.getWorldNormal();		// find face normal in world coordinates
                rayDirC2F.sub(fCent, clp);			// vector from centerline to face
                rayDirF2C = rayDirC2F.clone();
                rayDirF2C.scale(-1.0);

                dists[n] = fCent.distance(clp);
                theta[n] = fNorm.angle(rayDirF2C) - Math.PI/2.0;	// -90 < theta < 90
                if (dists[n] < dists[minLoc])
                    minLoc = n;

                //Intersector intersector = new Intersector();	// what is this?
                //Vector3d duv = new Vector3d();			// what is this? does it need to be new for each collision?

                //faceHit = geom.getBvHierarchy().nearestFaceIntersectedByRay(clp, rayDirC2F, duv, intersector); // --> calc ray to face, then use this
                //faceHit = geom.getObbtree().intersect(clp, rayDirC2F, duv, intersector); // --> calc ray to face, then use this
                //faceHit = geom.getBVTree().intersect(clp, rayDirC2F, duv, null); // --> calc ray to face, then use this
                //geom.getBVTree().i
                BVFeatureQuery query = new BVFeatureQuery();
                faceHit = query.nearestFaceAlongRay(null, null, geom, clp, rayDirC2F);

                if (faceHit == face)
                {
                    vis[n] = true;
                    if ((onlyNearest == false) && (Math.abs(theta[n]) > thetaMin) )
                    {
                        fsiFaces.add(face);
                        break;
                    }
                }
                else
                {
                    vis[n] = false;
                }
            }
            // in case only nearest is true
            if ( (vis[minLoc] == true) && (Math.abs(theta[minLoc]) > thetaMin) )
                fsiFaces.add(face);
        }

        fsiStructureFaces = fsiFaces;
    }

    public void writeFSIFaces(String filename)
    {
        PolygonalMesh geom = structureSolution.getGeometry();

        int nf = geom.getFaces().size();
        double[] isFSI = new double[nf];
        for (int a=0; a<nf; a++)
        {
            if (fsiStructureFaces.contains(geom.getFaces().get(a)) == true)
                isFSI[a] = 1.0;
            else
                isFSI[a] = 0.0;
        }
        VTK_IO.writeVTK(filename, geom);
    }

}