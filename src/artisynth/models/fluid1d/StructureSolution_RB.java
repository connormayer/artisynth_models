package artisynth.models.fluid1d;

import java.util.ArrayList;

import maspack.geometry.*;
import maspack.matrix.*;
import maspack.spatialmotion.Wrench;
import artisynth.core.mechmodels.*;
import artisynth.models.fluid1d.fileIO.VTK_IO;
import artisynth.models.fluid1d.fluidUtils.FluidUtils;

public class StructureSolution_RB implements StructureSolution 
{

    MechModel mechModel;
    RigidBody rigidBody;						// rigib bodies
    ArrayList<Marker> markers = new ArrayList<Marker>();		// markers
    double[] pressures;

    boolean writeBinary;

    public void initialize()
    {
    }

    public void clearExternalForces() 
    {
        rigidBody.zeroExternalForces();
    }

    public PolygonalMesh getGeometry() 
    {
        return rigidBody.getMesh();
    }

    public void setPressureOnFace(double pressure, Face face) 
    {
        // resolve pressure to a force which acts perpendicular to face (opposite of normal)
        Vector3d pForce = new Vector3d();
        pForce.scale( -1.0*pressure*FluidUtils.AreaOfFace(face), face.getWorldNormal() );
        setForceOnFace(pForce, face);

        //
        pressures[rigidBody.getMesh().getFaces().indexOf(face)] = pressure;
    }

    public void setForceOnFace(Vector3d force, Face face) 
    {
        Wrench wrench = new Wrench();
        Point3d pCent = new Point3d();
        face.computeCentroid(pCent);
        rigidBody.computeAppliedWrench(wrench, force, pCent);	// gets wrench in *body* coordinates
        wrench.transform(rigidBody.getPose().R);					// transform to TW coordinates
        wrench.add(rigidBody.getExternalForce());
        rigidBody.setExternalForce(wrench);					// this should over-write any previous forces
        //rigidBody.getExternalForce().add(wrench);				// works too, but I guess not a good idea

        //System.out.println("External forces: " + rigidBody.getExternalForce().toString());
        //System.out.println("Force: " + rigidBody.getForce().toString());
    }

    public Marker createMarkerPoint(Point3d point) 
    {
        // point needs to be in RB coordinates!
        FrameMarker fm = new FrameMarker(rigidBody, point);
        markers.add(fm);
        if (mechModel != null)
            mechModel.addFrameMarker(fm);
        return fm;
    }

    public Marker createMarkerPoint(Point3d point, Face face) 
    {
        return createMarkerPoint(point);
    }

    public ArrayList<Marker> getMarkers() 
    {
        return markers;
    }

    public void clearMarkers() 
    {
        for (Marker m : markers)
        {
            mechModel.removeFrameMarker((FrameMarker)m);
        }
    }

    public void setMechModel(MechModel mm)
    {
        mechModel = mm;
    }

    public void setRigidBody(RigidBody rb)
    {
        rigidBody = rb;
        pressures = new double[rb.getMesh().numFaces()];
    }

    public void writeSolution(String dir, String basename)
    {
        String filename = dir + basename + ".vtk";
        // option 1 - non-static write
        VTK_IO vtkIO = new VTK_IO();
        vtkIO.addPointDataScalars("pressure", pressures);
        vtkIO.write(filename, rigidBody.getMesh());
        // option 2 - static write without pressure data
        //artisynth.models.peterUtilities.fileIO.VTK_IO.writeVTK(filename, rigidBody.getMesh());
        // option 3 - the memory-leaking vtk library method
        //PeterVTKUtilities.vtkPolyData_Write(filename, PeterVTKUtilities.ArtisynthToVTK_SurfaceMesh(rigidBody.getMesh(), pressures), writeBinary);
    }

    public void writeSolution(String dir, String basename, String suffix)
    {
        writeSolution(dir, basename + suffix);
        //      String filename = dir + basename + suffix + ".vtk";
        //      PeterVTKUtilities.vtkPolyData_Write(filename, PeterVTKUtilities.ArtisynthToVTK_SurfaceMesh(rigidBody.getMesh(), pressures), writeBinary);
    }

    public void setWriteBinary(boolean useBinary)
    {
        writeBinary = useBinary;
    }


}
