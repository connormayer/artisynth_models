package artisynth.models.fluid1d;

public class FSISolver_WithCoupler implements FSISolver 
{
    // Using a coupler allows the preferred methods/settings to be used automatically

    FluidSolver fluidSolver;
    StructureSolver structureSolver;
    Coupler coupler;

    double t0=0, t1=0, t2=0, t3=0, t4=0, t40=0;
    double tSim = 0.0;

    public void step(double dt)
    {
        tSim = tSim + dt;
        System.out.println(String.format("--- Starting Step (t=%f) ---", tSim));

        t40=t4;
        t0=time_microseconds();
        structureSolver.step(dt);
        t1=time_microseconds();
        coupler.updateGeometry();
        t2=time_microseconds();
        fluidSolver.step(dt);
        t3=time_microseconds();
        coupler.updateForces();
        t4=time_microseconds();

        System.out.printf("Timing [ms]. SS: %4.3f,    Geom: %4.3f,    FS: %4.3f,    Force: %4.3f,    Other: %4.3f\n", t1-t0, t2-t1, t3-t2, t4-t3, t0-t40);
        System.out.println( String.format("--- Step Completed (t=%f) --- \n", tSim) );
    }
    double time_microseconds()
    {
        return ((double)System.nanoTime())/1000000.0;
    }

    public void initialize()
    {
        structureSolver.initialize();
        coupler.initialize();
        fluidSolver.initialize();

        //coupler.setFluidSolution(fluidSolver);
        //coupler.setStructureSolution(structureSolver.getSolution());

    }

    // ----
    public void setFluidSolver(FluidSolver fs)
    {
        fluidSolver = fs;
    }

    public void setStructureSolver(StructureSolver ss)
    {
        structureSolver = ss;
    }

    public void setCoupler(Coupler c)
    {
        coupler = c;
    }

    public FluidSolver getFluidSolver() 
    {
        return fluidSolver;
    }

    public StructureSolver getStructureSolver() 
    {
        return structureSolver;
    }

    public Coupler getCoupler()
    {
        return coupler;
    }

}
