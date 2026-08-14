package artisynth.models.fluid1d;

public interface FSISolver 
{  
    // setup and run
    public void initialize();  
    public void step(double dt);

    // basic setup functions
    public void setFluidSolver(FluidSolver fs);
    public void setStructureSolver(StructureSolver ss);
    public FluidSolver getFluidSolver();
    public StructureSolver getStructureSolver();
    //   public void SetCoupler(Coupler c);
}
