package artisynth.models.fluid1d;

import artisynth.core.modelbase.*;

public class FSIMonitor extends MonitorBase 
{
    // this class will be used to plug my FSISolver in as an artisynth monitor

    FSISolver fsiSolver;

    double time = 0.0;		// time in seconds
    double time0 = 0.0;
    double dt;			// time step (seconds)
    double tTol;

    // writing code
    String writeDir;
    String basename;
    boolean writeBinary = false;
    double dt_write = 0.1;	// write frequency
    int nWrites = 0;
    double lastWrite = -1000.0;	// last write time

    public void setFSISolver(FSISolver solver)
    {
        fsiSolver = solver;
    }

    public void setWriteBinary(boolean writeBinary)
    {
        this.writeBinary = writeBinary;
        fsiSolver.getStructureSolver().getSolution().setWriteBinary(writeBinary);
        // TODO: binary writing not implemented with fluid solver; this is not a nice way to implement this!

    }

    public void setSolutionWriting(double dtWrite, String outputDir, String basename)
    {
        writeDir = outputDir;
        this.basename = basename;
        dt_write = dtWrite;

        writeSolution(time);		// good to write out the initial conditions
        // boolean write = true ...
    }

    public void apply(double time0, double time)
    {
        // find time and dt
        dt = time - time0;
        tTol = dt/1000.0;

        fsiSolver.step(dt);
        if (writeDir != null)
            writeSolution(time);
    }

    public void writeSolution(double t)
    {
        if ( t-lastWrite + tTol >= dt_write )
        {
            //String dir = "/home/peter/research/simulations/fsiTests/couplerTests/";
            //String filename = String.format ("struct_%04d_%1.3f", nWrites, t);		// peter friendly
            //String filename = String.format ("%s_%04d", basename, nWrites);			// paraview friendly
            String suffix = String.format ("_%06d", nWrites);				// paraview friendly
            fsiSolver.getStructureSolver().getSolution().writeSolution(writeDir, basename, suffix);
            fsiSolver.getFluidSolver().getSolution().writeSolution(writeDir, basename + "_fluid" + suffix );

            lastWrite = t;
            nWrites++;

            System.out.println("Last write time = " + t);
        }
    }

    // TODO: this doesn't work...there are some access issues (but maybe worth spending some more time on)
    //   public void writeClassState(Object model, String filename)
    //   {
    //      try
    //      {
    //	 PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
    //	 int nFields = model.getClass().getDeclaredFields().length;
    //	 for (int a=0; a<nFields; a++)
    //	 {
    //	    if (model.getClass().getDeclaredFields()[a].getType().isPrimitive() == true)
    //	    {
    //	       file.println(model.getClass().getDeclaredFields()[a].getName() + " = " + model.getClass().getDeclaredFields()[a].get(model));
    //	    }
    //	 }
    //	 file.close();
    //      }
    //      catch(Exception e)
    //      {
    //	 e.printStackTrace();
    //      }
    //   }

}

//class OutputMonitor extends MonitorBase
//{
//   String dir;
//   
//   double t_prev = 0.0;
//   double dt_write = 1.0;
//
//   public OutputMonitor(String dir)
//   {
//      this.dir = dir;
//      
//      File dataDir = new File(dir);
//      if (dataDir.exists() == false)
//      {
//         dataDir.mkdir();
//      }
//   }
//
//   public void apply (long t)
//   {
//      double t_sec = ((double)t)/1000000000.0;          // time in seconds
//      
//      if (t_sec-t_prev >= dt_write )
//      {
//         String filename = String.format ("%s_mesh_%1.3f.vtk", fem.getName(), t_sec);
//         PeterVTKUtilities.vtkUnstructuredGrid_Write (dir + filename, PeterVTKUtilities.ArtisynthToVTK_Mesh (fem));
//         
//         t_prev = t_sec;
//      }
//   }
//}
