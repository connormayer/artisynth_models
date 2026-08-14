package artisynth.models.fluid1d;

public class FluidSolver_1D_Dummy implements FluidSolver_1D
{    
    // A dummy fluid solver for the case that the user want to define their own fluid soln (analytical cases, etc...)
    FluidSolution_1D soln;

    public void initialize() 
    {    
    }

    public void step(double dt) 
    {    
    }
    
    public void setSolution(FluidSolution_1D solution)
    {
        soln = solution;
    }
    
    public FluidSolution_1D getSolution()
    {
        return soln;
    }
}
