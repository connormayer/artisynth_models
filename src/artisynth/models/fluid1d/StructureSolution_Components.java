package artisynth.models.fluid1d;

import java.util.ArrayList;
import artisynth.core.femmodels.*;
import artisynth.core.mechmodels.*;
import artisynth.models.fluid1d.fileIO.VTK_IO;
import maspack.geometry.*;
import maspack.matrix.*;

public class StructureSolution_Components implements StructureSolution
{

    ArrayList<StructureSolution_FEM> fems = new ArrayList<StructureSolution_FEM>();		// fem models
    ArrayList<StructureSolution_RB> rbs   = new ArrayList<StructureSolution_RB>();		// rigib bodies

    PolygonalMesh geometry;
    // create lists to link the partial geometries to the master geometry ... it ain't pretty, but gets the job done
    ArrayList<Face> facesPieces = new ArrayList<Face>();					// faces from individual components
    ArrayList<Face> facesMaster = new ArrayList<Face>();					// faces in master geometry
    ArrayList<StructureSolution> facesSolns = new ArrayList<StructureSolution>();

    boolean writeBinary = false;


    public void initialize()
    {
        updateGeometry();
    }

    public void addFEM(FemModel3d fem)
    {
        StructureSolution_FEM femSoln = new StructureSolution_FEM();
        femSoln.setFemModel(fem);
        fems.add(femSoln);
    }

    public void addRigidBody(RigidBody rb, MechModel mm)
    {
        StructureSolution_RB rbSoln = new StructureSolution_RB();
        rbSoln.setRigidBody(rb);
        rbSoln.setMechModel(mm);
        rbs.add(rbSoln);
    }

    public void clearExternalForces()
    {
        for (StructureSolution_FEM fs : fems)
            fs.clearExternalForces();
        for (StructureSolution_RB rs : rbs)
            rs.clearExternalForces();
    }

    //   Face findNearestFace(PolygonalMesh mesh, Point3d point)
    //   {
    //      Point3d pLocal = new Point3d();
    //      pLocal.inverseTransform(mesh.getMeshToWorld(), point);
    //
    //      // consider the bounds
    //      //mesh.clearObbtree();
    //      //mesh.getObbtree().
    //      AjlBvTree bvTree = mesh.getBvHierarchy();
    //      Point3d ptMin = new Point3d();
    //      Point3d ptMax = new Point3d();
    //      bvTree.updateBounds(ptMin, ptMax);
    //      
    //      Point3d plMin = new Point3d();
    //      Point3d plMax = new Point3d();
    //      mesh.getLocalBounds(plMin, plMax);
    //      
    //      Point3d pwMin = new Point3d();
    //      Point3d pwMax = new Point3d();
    //      mesh.getWorldBounds(pwMin, pwMax);
    //
    //      Point3d loc = new Point3d();
    //      Face f1 = mesh.getBvHierarchy().nearestFaceToPoint(pLocal, new Vector3d(), loc, new Vector2d(), new Intersector());
    //      Face f = mesh.getObbtree().nearestFace(pLocal, new Vector3d(), loc, new Vector2d(), new Intersector());
    //
    //      return f;
    //   }
    //   
    //   public Marker createMarkerPoint_temp(Point3d point) 
    //   {
    //      // should I work in local or world here?!?!?!
    //      double dMin = 1000.0;
    //      Face fMin = null;
    //      StructureSolution sMin = null;
    //      
    //      for ( StructureSolution_FEM fem : fems )
    //      {
    //	 // temp
    //	 Face fTemp = findNearestFace(fem.getGeometry(), point);
    //	 
    //	 Point3d pLocal = new Point3d();
    //	 pLocal.inverseTransform(fem.getGeometry().getMeshToWorld(), point);
    //	 
    //	 Point3d loc = new Point3d();
    //	 AjlBvTree bvTree = fem.getGeometry().getBvHierarchy();
    //	 //Face f = fem.getGeometry().getBvHierarchy().nearestFaceToPoint(pLocal, new Vector3d(), loc, new Vector2d(), new Intersector());
    //	 Face f = fem.getGeometry().getObbtree().nearestFace(pLocal, new Vector3d(), loc, new Vector2d(), new Intersector());
    //	 
    //	 double dist = loc.distance(point);
    //	 if (dist < dMin)
    //	 {
    //	    dMin = dist;
    //	    fMin = f;
    //	    sMin = fem;
    //	 }
    //      }
    //      for ( StructureSolution_RB rb : rbs )
    //      {
    //	 //temp
    //	 Face fTemp = findNearestFace(rb.getGeometry(), point);
    //	 
    //	 
    //	 Point3d pLocal = new Point3d();
    //	 pLocal.inverseTransform(rb.getGeometry().getMeshToWorld(), point);
    //	 
    //	 Point3d loc = new Point3d();
    //	 //Face f = rb.getGeometry().getBvHierarchy().nearestFaceToPoint(point, new Vector3d(), loc, new Vector2d(), new Intersector());
    //	 Face f = rb.getGeometry().getObbtree().nearestFace(pLocal, new Vector3d(), loc, new Vector2d(), new Intersector());
    //	 
    //	 double dist = loc.distance(point);
    //	 if (dist < dMin)
    //	 {
    //	    dMin = dist;
    //	    fMin = f;
    //	    sMin = rb;
    //	 }
    //      }
    //      
    //      return sMin.createMarkerPoint(point, fMin);
    //   }

    public Marker createMarkerPoint(Point3d point) 
    {
        Point3d nearest = new Point3d();
        //Face face = geometry.getBvHierarchy().nearestFaceToPoint(point, new Vector3d(), nearest, new Vector2d(), new Intersector());
        //Face face = geometry.getObbtree().nearestFace(point, new Vector3d(), nearest, new Vector2d(), new Intersector());
        Face face = BVFeatureQuery.getNearestFaceToPoint(null, null, geometry, point);
        //Face face = geometry.getObbtree().nearestFace(point, null, nearest, new Vector2d(), new Intersector());

        return createMarkerPoint(point, face);
    }

    public Marker createMarkerPoint(Point3d point, Face face) 
    {
        int loc = facesMaster.indexOf(face);
        return facesSolns.get(loc).createMarkerPoint(point, facesPieces.get(loc));
    }

    public ArrayList<Marker> getMarkers() 
    {
        ArrayList<Marker> markers = new ArrayList<Marker>();
        for (StructureSolution_FEM fs : fems)
            markers.addAll(fs.getMarkers());
        for (StructureSolution_RB rs : rbs)
            markers.addAll(rs.getMarkers());

        return markers;
    }

    public void clearMarkers() 
    {
        for (StructureSolution_FEM fs : fems)
            fs.clearMarkers();
        for (StructureSolution_RB rs : rbs)
            rs.clearMarkers();
    }

    public void setForceOnFace(Vector3d force, Face face) 
    {
        int loc = facesMaster.indexOf(face);
        facesSolns.get(loc).setForceOnFace(force, facesPieces.get(loc));
    }

    public void setPressureOnFace(double pressure, Face face) 
    {
        int loc = facesMaster.indexOf(face);
        if (loc == -1)
            System.out.println("Structure Solution: Face not found in master index.");
        else
            facesSolns.get(loc).setPressureOnFace(pressure, facesPieces.get(loc));
    }

    public void clearGeometry()
    {
        geometry = new PolygonalMesh();
    }

    public PolygonalMesh getGeometry() 
    {
        //updateGeometry();	// really, why would I do this?
        return geometry;
    }

    void updateGeometry()
    {
        clearGeometry();
        for (StructureSolution_FEM fs : fems)
            addGeometry(fs.getGeometry(), fs);
        for (StructureSolution_RB rs : rbs)
            addGeometry(rs.getGeometry(), rs);
    }

    public void addGeometry(PolygonalMesh mesh, StructureSolution soln)
    {
        // first import the vertices
        Vertex3d[] newVertices = new Vertex3d[mesh.numVertices()];
        for (int a=0; a<mesh.numVertices(); a++)
        {
            Vertex3d vNew = new Vertex3d(mesh.getVertices().get(a).getWorldPoint());
            //Vertex3d vNew = new Vertex3d(mesh.getVertices().get(a).getPosition());
            newVertices[a] = vNew;
            geometry.addVertex(vNew);
        }

        // faces
        for (Face f : mesh.getFaces())
        {
            Vertex3d[] fv = new Vertex3d[f.getVertexIndices().length];

            int index = 0;
            for (int b=0; b<fv.length; b++)
            {
                index = mesh.getVertices().indexOf(f.getVertex(b));		// index of the old vertex
                fv[b] = newVertices[index];					// add the new vertex
            }
            Face fNew = geometry.addFace(fv);

            facesPieces.add(f);
            facesMaster.add(fNew);
            if (soln != null)
                facesSolns.add(soln);
        }

        //MeshFactory.createBox()
        //geometry.updateBounds();
        geometry.updateFaceNormals();
    }

    public Face getMasterFromComponent(Face face)
    {
        int loc = facesPieces.indexOf(face);
        return facesMaster.get(loc);
    }
    public Face getComponentFromMaster(Face face)
    {
        int loc = facesMaster.indexOf(face);
        return facesPieces.get(loc);
    }

    public void writeSolution(String dir, String basename)
    {
        for (int a=0; a<fems.size(); a++)
        {
            fems.get(a).writeSolution(dir, basename + "_fem" + a);
        }
        for (int a=0; a<rbs.size(); a++)
        {
            rbs.get(a).writeSolution(dir, basename + "_rb" + a);
        }
    }

    public void writeSolution(String dir, String basename, String suffix)
    {
        for (int a=0; a<fems.size(); a++)
        {
            fems.get(a).setWriteBinary(writeBinary);	// TODO: not nice, but safer than assigning in the function below...
            fems.get(a).writeSolution(dir, basename + "_fem" + a + suffix);
        }
        for (int a=0; a<rbs.size(); a++)
        {
            rbs.get(a).setWriteBinary(writeBinary);
            rbs.get(a).writeSolution(dir, basename + "_rb" + a + suffix);
        }
    }

    public void setWriteBinary(boolean useBinary)
    {
        writeBinary = useBinary;
    }

    public void writeCompositeGeometry(String dir, String filename)
    {
        double[] modelCode = new double[facesSolns.size()];
        for (int a=0; a<facesSolns.size(); a++)
        {
            modelCode[a] = (double)facesSolns.get(a).hashCode();
        }
        VTK_IO.writeVTK(filename, geometry);
        //artisynth.models.peterUtilities.PeterVTKUtilities.vtkPolyData_Write(dir + filename, artisynth.models.peterUtilities.PeterVTKUtilities.ArtisynthToVTK_SurfaceMesh(geometry, modelCode));
    }

}
