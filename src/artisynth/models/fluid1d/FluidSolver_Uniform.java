package artisynth.models.fluid1d;

public class FluidSolver_Uniform implements FluidSolver 
{
    // Provides a uniform pressure
    // u(x,t) = 0	--> the fluid is still
    // p(x,t) = p(t)  	--> uniform in space, can be varying in time
    // doesn't need a geometry or mesh

    FluidSolution_Uniform soln;
    double t;

    double p_uniform;

    public void initialize()
    {
        t = 0.0;
        soln.setPressure(p_uniform);
    }

    public void step(double dt)
    {
        // no updates needed, unless p = p(t)
        t = t + dt;
        soln.setPressure(p_uniform);
    }

    // --- Class Specific Implementations --- //

    public void setSolution(FluidSolution_Uniform solution)
    {
        soln = solution;
    }

    public FluidSolution_Uniform getSolution()
    {
        return soln;
    }

    public void setUniformPressure(double pressure)
    {
        p_uniform = pressure;
    }

}
