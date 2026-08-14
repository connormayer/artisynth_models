package artisynth.models.fluid1d;

public class FSISolver_LooselyCoupled implements FSISolver 
{
    // This FSI solver can be used if non-standard force/geom couplers are being used and want to be added by hand

    FluidSolver fluidSolver;
    StructureSolver structureSolver;
    GeometryCoupler geometryCoupler;
    ForceCoupler forceCoupler;

    public void step(double dt)
    {
        structureSolver.step(dt);
        geometryCoupler.update();
        fluidSolver.step(dt);
        forceCoupler.update();
    }

    public void initialize()
    {
    }

    // Implementations
    public void setFluidSolver(FluidSolver fs)
    {
        fluidSolver = fs;
    }

    public void setStructureSolver(StructureSolver ss)
    {
        structureSolver = ss;
    }

    public FluidSolver getFluidSolver() 
    {
        return fluidSolver;
    }

    public StructureSolver getStructureSolver() 
    {
        return structureSolver;
    }

    public void setGeometryCoupler(GeometryCoupler gi)
    {
        geometryCoupler = gi;
    }

    public void setForceCoupler(ForceCoupler fi)
    {
        forceCoupler = fi;
    }

}
