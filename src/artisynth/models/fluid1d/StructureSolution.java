package artisynth.models.fluid1d;

import java.util.ArrayList;
import artisynth.core.mechmodels.*;
import maspack.geometry.*;
import maspack.matrix.*;

/*
 A structure solution can be a FEM or a RB, so there is no guarantee that it contains 
 1) a volume mesh 
 2) a solution inside the object (that is, on the mesh)
 but it does contain:
 1) a surface mesh
 2) a solution on the surface (pressures/forces in this case) 
 */

public interface StructureSolution 
{
    // basics
    public void initialize();
    // update()  ?


    // --- geometric methods --- //
    public PolygonalMesh getGeometry();


    // --- surface solution methods --- //
    // global
    public void clearExternalForces();
    // local
    public void setPressureOnFace(double pressure, Face face);
    public void setForceOnFace(Vector3d force, Face face);
    // get Pressure/Force ??		--> these are tricky to implement p/f gets interpolated to nodes for fems


    // --- marker point methods --- //
    public Marker createMarkerPoint(Point3d point);
    public Marker createMarkerPoint(Point3d point, Face face);			// hmmmmm
    //public void setMarkerPoints(ArrayList<Point3d> point, Face face);
    public ArrayList<Marker> getMarkers();
    public void clearMarkers();

    // --- output --- //
    public void writeSolution(String dir, String basename);
    public void writeSolution(String dir, String basename, String suffix);
    public void setWriteBinary(boolean writeBinary);



}
