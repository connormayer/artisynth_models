package artisynth.models.fluid1d;

import java.util.ArrayList;
import maspack.geometry.*;


public class ForceCoupler_Direct implements ForceCoupler 
{
    // A direct coupling takes a known pressure and applies it to all the given FSI faces. Very Simple. No interpolation needed.

    FluidSolution_Uniform fluidSolution;
    StructureSolution structureSolution;

    ArrayList<Face> targetFaces;

    public void initialize()
    {
    }

    public void update()
    {
        double pressure = fluidSolution.getPressure();

        structureSolution.clearExternalForces();
        for (Face f: targetFaces)
        {
            structureSolution.setPressureOnFace(pressure, f);
        }
    }

    public void setFluidSolution(FluidSolution_Uniform fs)
    {
        fluidSolution = fs;
    }

    public void setStructureSolution(StructureSolution ss)
    {
        structureSolution = ss;
    }

    public void setTargetFaces(ArrayList<Face> faces)
    {
        targetFaces = faces;
    }
}
