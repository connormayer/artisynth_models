package artisynth.models.fluid1d;

public interface FluidSolver 
{
    public void initialize();
    public void step(double dt);

    public FluidSolution getSolution();
    //public void setSolution(FluidSolution soln);	// I'd like to enforce this...

    //
    // get/set geometry
    // get solution
    // get/set equations

}
