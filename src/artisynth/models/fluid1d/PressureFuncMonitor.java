package artisynth.models.fluid1d;

import java.io.*;

import artisynth.core.driver.Main;
import artisynth.core.modelbase.MonitorBase;

public class PressureFuncMonitor extends MonitorBase  
{
    //   boolean twoWay = false;
    //   double T0 = 0.0;
    //   double T1;
    //   double p0;
    //   double p1;
    FluidSolver_1D fluidSolver;

    double time;
    double pressure;

    double[] t;
    double[] p;
    int n = 0;
    int nLocs;

    String outputFile;
    PrintWriter file;

    public PressureFuncMonitor(double[] tArr, double[] pArr, FluidSolver_1D fluidSolver)
    {
        t = tArr;
        p = pArr;

        nLocs = t.length;
        this.fluidSolver = fluidSolver;

        setPressure(p[0]);
    }

    public void writeLog(String filename)
    {
        outputFile = filename;
        createLog();
    }

    public void apply(double t0, double t1)
    {
        time = t1; //0.5*(t0+t1);

        while (time > t[n])
        {
            n++;
            if (n>=nLocs)
            {
                Main.exit(0);
                //getModel().dispose();	// how do I stop the simulation from here?? this works, but...
            }
            
        }

        pressure = ((p[n] - p[n-1])/(t[n] - t[n-1]))*(time-t[n]) + p[n];

        setPressure(pressure);
        System.out.printf("Time = %f, pressure = %f \n", time, pressure);
        writeLog();

    }

    void setPressure(double p)
    {
        if (fluidSolver instanceof FluidSolver_1D_Uniform)
            ((FluidSolver_1D_Uniform)fluidSolver).setUniformPressure(p);
        else if (fluidSolver instanceof FluidSolver_1D_NSE)
            ((FluidSolver_1D_NSE)fluidSolver).setEqn_Uniform(0.0, p, 0.0);
    }

    void createLog()
    {
        // log: t, p, a0...aN
        try
        {
            file = new PrintWriter(new BufferedWriter(new FileWriter(outputFile, false)));
            file.print("time");
            file.print(",pressure");
            for (int a=0; a<fluidSolver.getSolution().getNumberOfPoints(); a++)
            {
                file.printf(",area%d", a);
            }
            file.println();
        }
        catch (Exception e)
        {
        }
    }

    void writeLog()
    {
        file.print(time);
        file.print("," + pressure);
        for (int a=0; a<fluidSolver.getSolution().getNumberOfPoints(); a++)
        {
            file.printf(",%09.8f", fluidSolver.getSolution().getGeometry().getArea(a));
        }
        file.println();
        file.flush();
    }

} 
