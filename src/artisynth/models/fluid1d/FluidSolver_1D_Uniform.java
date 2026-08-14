package artisynth.models.fluid1d;

public class FluidSolver_1D_Uniform implements FluidSolver_1D 
{
    FluidSolution_1D soln;
    double t;
    double p_uniform;

    public void initialize()
    {
        step(1.0);
        t = 0.0;
    }

    public void step(double dt)
    {
        t = t + dt;

        for (int a=0; a<soln.getNumberOfPoints(); a++)
        {
            soln.setField("u", a, 0.0);
            soln.setField("p", a, p_uniform);
        }

        System.out.printf( "Geometry is %3.2f percent closed\n", soln.getGeometry().percentClosed() );
    }

    //
    public void setUniformPressure(double p)
    {
        p_uniform = p;
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
