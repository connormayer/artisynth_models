package artisynth.models.fluid1d;

public interface Coupler 
{
    public void initialize();
    public void updateGeometry();
    public void updateForces();

    // I should enforce this...
    //public void setFluidSolution(FluidSolution fs);
    //public void setStructureSolution(StructureSolution ss);

}
