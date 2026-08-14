package artisynth.models.fluid1d;

public interface ForceCoupler 
{
    public void initialize();
    public void update();

    // --- Implementations --- //
    //public void setFluidSolution(FluidSolver fs);
    //public void setStructureSolution(StructureSolver ss);
    //public void setTargetFaces(ArrayList<Face> faces);

}
