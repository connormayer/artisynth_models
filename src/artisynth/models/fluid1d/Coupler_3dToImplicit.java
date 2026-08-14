package artisynth.models.fluid1d;

import java.util.ArrayList;
import maspack.matrix.*;
import maspack.geometry.*;

public class Coupler_3dToImplicit implements Coupler 
{  
    GeometryCoupler_Dummy geometryCoupler = new GeometryCoupler_Dummy();		// geometry coupling
    ForceCoupler_Direct forceCoupler = new ForceCoupler_Direct();		// force coupling

    StructureSolution structureSolution;			// structure solution
    FluidSolution_Uniform fluidSolution;					// fluid solution

    ArrayList<Face> fsiStructureFaces;				// should this be a geometry_3d?
    //ArrayList<Face> fsiFluidFaces;

    public void initialize()
    {
        // build up the geometry and force couplers here
        //geometryCoupler.setInputSolution(structureSolution);
        //geometryCoupler.setOutputSolution(fluidSolution);
        geometryCoupler.initialize();

        forceCoupler.setFluidSolution(fluidSolution);
        forceCoupler.setStructureSolution(structureSolution);
        forceCoupler.setTargetFaces(fsiStructureFaces);			// must be defined!

    }

    // generic caluclation --> not easy
    public void findFSISurfaces()
    {
    }

    // provided from the user knowledge of the problem --> better, easier
    public void setFSIStructureFaces(ArrayList<Face> fsiFaces)
    {
        fsiStructureFaces = fsiFaces;
    }


    public void updateGeometry()
    {
        //geometryCoupler.SetInterpolationPoints(MarkersToPoints(intersectionMarkers));		// is this update needed?
        geometryCoupler.update();
    }

    public void updateForces()
    {
        //forceCoupler.SetInterpolationPoints(MarkersToPoints(intersectionMarkers));
        forceCoupler.update();
    }

    public void setFluidSolution(FluidSolution_Uniform fs)
    {
        fluidSolution = fs;
    }

    public void setStructureSolution(StructureSolution ss)
    {
        structureSolution = ss;
    }

}