package artisynth.models.fluid1d;

import java.util.ArrayList;
import artisynth.core.femmodels.*;
import artisynth.core.mechmodels.*;
import maspack.geometry.*;
import maspack.matrix.*;

public class StructureSolution_FEM implements StructureSolution
{
    FemModel3d fem;		// fem model
    ArrayList<Marker> markers = new ArrayList<Marker>();
    boolean writeBinary = false;


    public void initialize()
    {
    }

    public void setFemModel(FemModel3d fem)
    {
        this.fem = fem;
    }

    // --- acting upon a single face --- //

    public void setPressureOnFace(double p, Face f)
    {
        // the pressure acts perpendicular to face (opposite of normal)
        Vector3d pForce = new Vector3d();
        //pForce.scale( -1.0*p*PeterUtilities.AreaOfFace(f), f.getNormal() );                     // resolve the pressure to a force
        pForce.scale( -1.0*p*f.computeArea(), f.getWorldNormal() );                     // resolve the pressure to a force

        setForceOnFace(pForce, f);                                             // apply the force
    }

    // apply a force for a face (by distributing evenly over the nodes)
    public void setForceOnFace(Vector3d force, Face f)
    {
        int numV = f.numVertices();
        Vector3d forcePerNode = new Vector3d();
        forcePerNode.scale (1.0/((double)numV), force);

        for (int a=0; a<numV; a++)
        {
            FemNode3d node3d = fem.getSurfaceNode(f.getVertex(a));
            //FemNode3d node3d = (FemNode3d)( ((FemMeshVertex)f.getVertex(a)).getPoint() );         
            node3d.getExternalForce().add(forcePerNode);
        }
    }

    // --- act upon all faces --- //
    public void clearExternalForces()
    {
        for (FemNode3d node : fem.getNodes() )
            node.setExternalForce(new Vector3d());
    }


    public PolygonalMesh getGeometry() 
    {
        return fem.getSurfaceMesh();
    }

    // --- Marker Code --- //
    public Marker createMarkerPoint(Point3d point) 
    {
        //      Point3d pTrans = new Point3d(point.x, point.y, point.z);
        //      pTrans.inverseTransform(fem.getSurfaceMesh().getMeshToWorld());

        FemMarker marker = new FemMarker(point);

        //      Point3d newLoc1 = new Point3d();
        //      Point3d newLoc2 = new Point3d();
        //      FemElement elem1 = fem.findNearestElement(newLoc1, point);
        //      FemElement elem2 = fem.findNearestSurfaceElement(newLoc2, point);

        markers.add(marker);
        fem.addMarker(marker);
        return marker;
    }

    public Marker createMarkerPoint(Point3d point, Face face) 
    {
        FemElement3dBase elem = fem.getSurfaceElement(face);
        if (elem != null)
        {
            FemMarker marker = new FemMarker(point);
            markers.add(marker);
            fem.addMarker(marker, elem);	// this can be much faster than fem.addMarker(marker) and should be used when possible.
            return marker;
        }
        else
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
            fem.removeMarker((FemMarker)m);
        }
    }

    public void writeSolution(String dir, String basename)
    {
        String filename = dir + basename + ".vtk";
        artisynth.models.fluid1d.fileIO.VTK_IO.writeVTK(filename, fem);
        // TODO: can I gather pressures and show as nodal forces??
        //PeterVTKUtilities.vtkUnstructuredGrid_Write(filename, PeterVTKUtilities.ArtisynthToVTK_Mesh(fem), writeBinary);
    }

    public void writeSolution(String dir, String basename, String suffix)
    {
        writeSolution(dir, basename + suffix);
        //      String filename = dir + basename + suffix + ".vtk";
        //      artisynth.models.peterUtilities.fileIO.VTK_IO.writeVTK(filename, fem);
        //PeterVTKUtilities.vtkUnstructuredGrid_Write(filename, PeterVTKUtilities.ArtisynthToVTK_Mesh(fem), writeBinary);
    }

    public void setWriteBinary(boolean useBinary)
    {
        writeBinary = useBinary;
    }


    // --- maybe this code should be cut --> keep it simple! --- //
    public void clearExternalForcesOnFaces(ArrayList<Face> faces)
    {
        for (FemNode3d node : getFaceNodes(faces))
        {
            node.setExternalForce(new Vector3d());
        }
    }

    // applying pressures to faces
    public void addPressuresToFaces(double[] pressures, ArrayList<Face> faces)
    {
        for (int a=0; a<pressures.length; a++)
        {
            setPressureOnFace(pressures[a], faces.get(a));
        }
    }
    public void setPressuresOnFaces(double[] pressures, ArrayList<Face> faces)
    {
        clearExternalForces();
        addPressuresToFaces(pressures, faces);
    }

    // applying forces to faces
    public void addForcesToFaces(ArrayList<Vector3d> forces, ArrayList<Face> faces)
    {
        for (int a=0; a<forces.size(); a++)
        {
            setForceOnFace(forces.get(a), faces.get(a));
        }
    }
    public void setForcesOnFaces(ArrayList<Vector3d> forces, ArrayList<Face> faces)
    {
        clearExternalForces();
        addForcesToFaces(forces, faces);
    }

    // applying forces to nodes
    public void addForcesToNodes(ArrayList<Vector3d> forces, ArrayList<FemNode3d> nodes)
    {
        for (int a=0; a<forces.size(); a++)
        {
            nodes.get(a).setExternalForce(forces.get(a));		// I think this removes all previous forces anyhow.
        }
    }
    public void setForcesOnNodes(ArrayList<Vector3d> forces, ArrayList<FemNode3d> nodes)
    {
        clearExternalForces();
        addForcesToNodes(forces, nodes);
    }

    public ArrayList<FemNode3d> getFaceNodes(ArrayList<Face> faces)
    {
        ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
        for (Face f : faces)
        {
            for (int a=0; a<f.numVertices (); a++)
            {
                FemNode3d node = fem.getSurfaceNode(f.getVertex(a));
                //FemNode3d node = (FemNode3d)( ((FemMeshVertex)f.getVertex(a)).getPoint() );
                if (nodes.contains (node) == false)
                {
                    nodes.add(node);
                }
            }
        }
        return nodes;
    }
    // end cuts...


}
