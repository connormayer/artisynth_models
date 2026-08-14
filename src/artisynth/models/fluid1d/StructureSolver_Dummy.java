package artisynth.models.fluid1d;

import artisynth.core.modelbase.*;
import artisynth.core.workspace.RootModel;

public class StructureSolver_Dummy implements StructureSolver
{
    // Because I'm using the FSI module as a monitor in Artisynth, this structure solver merely gathers the solution from Artisynth
    RootModel model;
    StructureSolution solution;

    boolean rbModel = false;
    boolean femModel = false;


    // this needs to go
    //ArrayList<FemModel3d> fems = new ArrayList<FemModel3d>();		// fem models
    //MechModel mechmod;		// rigib bodies

    public StructureSolver_Dummy()
    {
    }

    public void initialize()
    {      
    }

    public void step(double dt)
    {
    }

    public void setSolution(StructureSolution soln)
    {
        solution = soln;
    }

    public StructureSolution getSolution()
    {
        return solution;
    }

    //   public void initializeComponents()
    //   {
    //      Geometry_3d geom = new Geometry_3d();
    //      StructureSolution_Components soln = new StructureSolution_Components();
    //      //soln.setGeometry(geom);
    //      //soln.initialize();
    //      
    //      // step through the model and find all FEMS and RBS
    //      for (int i=0; i<model.models().size(); i++)
    //      {
    //	 if ( model.models().get(i).getClass().equals(FemModel3d.class) == true )
    //	 {
    //	    FemModel3d fem = (FemModel3d)model.models().get(i);
    //	    soln.addFEM(fem);
    //	    fems.add(fem);
    //	 }
    //	 else if ( model.models().get(i).getClass().equals(MechModel.class) == true )
    //	 {
    //	    // check if dynamic?
    //	    MechModel mm = (MechModel)model.models().get(i);
    //	    mechmod = mm;
    //	    for (RigidBody rb : mm.rigidBodies())
    //	    {
    //	       soln.addRigidBody(rb, mm);
    //	    }
    //	 }
    //      }
    //      
    //      // add my structure markers     
    //      solution = soln;
    //   }


    //   public void setStructure(FemModel3d fem)
    //   {
    //      // this can go...
    //      femModel = true;
    //      StructureSolution_FEM soln = new StructureSolution_FEM();
    //      soln.setFemModel(fem);
    //      solution = soln;
    //   }
    //   
    //   public void setStructure(RigidBody rb, MechModel mm)
    //   {
    //      // this can go too...
    //      rbModel = true;
    //      StructureSolution_RB soln = new StructureSolution_RB();
    //      soln.setRigidBody(rb);
    //      soln.setMechModel(mm);
    //      solution = soln;
    //   }

}
