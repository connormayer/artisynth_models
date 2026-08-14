package artisynth.models.fluid1d;

public interface FluidSolver_1D extends FluidSolver
{      
    public void setSolution(FluidSolution_1D solution);
    public FluidSolution_1D getSolution();
}
