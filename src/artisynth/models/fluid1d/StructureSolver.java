package artisynth.models.fluid1d;

public interface StructureSolver 
{
    public void initialize();
    public void step(double dt);

    public StructureSolution getSolution();

    //public void writeSolution(String dir, String basename);
}
