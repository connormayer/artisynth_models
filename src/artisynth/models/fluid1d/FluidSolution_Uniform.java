package artisynth.models.fluid1d;

import java.io.*;

public class FluidSolution_Uniform implements FluidSolution
{
    // my solution variables
    double p;
    double u;

    public void initialize()
    {
    }

    public double getPressure()
    {
        return p;
    }

    public double getVelocity()
    {
        return u;
    }

    public void setPressure(double pressure)
    {
        p = pressure;
    }

    public void setVelocity(double velocity)
    {
        u = velocity;
    }

    public void writeSolution(String dir, String filename)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(dir + filename, false)));

            file.print("u,p\n");		//heh...
            file.printf("%f,%f\n", u, p);
            file.close();
        }
        catch(Exception e)
        {
        }
    }

}
