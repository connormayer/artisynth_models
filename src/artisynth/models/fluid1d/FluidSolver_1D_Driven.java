package artisynth.models.fluid1d;

public class FluidSolver_1D_Driven implements FluidSolver_1D 
{
    FluidSolution_1D soln;
    double t;
    DrivingFunction df;

    public void initialize()
    {
        t = 0.0;
        //soln = new FluidSolution_1D();
        step(0.0);
    }

    public void step(double dt)
    {
        t = t + dt;
        df.update(t, soln);
    }

    public void setDrivingFunction(DrivingFunction df)
    {
        this.df = df;
    }

    public void setSolution(FluidSolution_1D solution)
    {
        soln = solution;
    }

    public FluidSolution_1D getSolution()
    {
        return soln;
    }

    // ------------- //
    public static class DrivingFunction
    {
        double p0;	// pressure left  of the x[i] at t[i]
        double p1;	// pressure right of the x[i] at t[i]
        double t[];
        double x[];
        int nLocs;

        public void update(double time, FluidSolution_1D soln)
        {
            int n=1;
            while (time > t[n])
            {
                n++;
                if (n>=nLocs)
                {
                    n=nLocs-1;
                    break;
                }
            }

            double ex = ((x[n] - x[n-1])/(t[n] - t[n-1]))*(time-t[n]) + x[n];

            for (int i=0; i<soln.getNumberOfPoints(); i++)
            {
                if (soln.getGeometry().getCenterline().getS(i) < ex)
                {
                    soln.setField("u", i, 0.0);
                    soln.setField("p", i, p0);
                    soln.setField("tau", i, 0.0);
                }
                else
                {
                    soln.setField("u", i, 0.0);
                    soln.setField("p", i, p1);
                    soln.setField("tau", i, 0.0);
                }
            }
        }

        public void setParameters(double p0, double p1, double[] t, double[] x)
        {
            this.p0 = p0;
            this.p1 = p1;
            this.t = t;
            this.x = x;
            nLocs = t.length;
        }
    }

}
