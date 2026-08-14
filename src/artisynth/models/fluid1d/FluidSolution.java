package artisynth.models.fluid1d;

public interface FluidSolution 
{
    public void initialize();
    public void writeSolution(String dir, String filename);
}
