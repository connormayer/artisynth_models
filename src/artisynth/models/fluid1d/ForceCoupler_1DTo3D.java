package artisynth.models.fluid1d;

import java.util.ArrayList;

import maspack.geometry.*;

public abstract class ForceCoupler_1DTo3D implements ForceCoupler
{
    FluidSolution_1D fluidSolution;
    StructureSolution structureSolution;

    ArrayList<Face> targetFaces;		// all the faces to interpolate to

    public abstract void initialize();

    public abstract void update();


    public void setFluidSolution(FluidSolution_1D fs)
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
