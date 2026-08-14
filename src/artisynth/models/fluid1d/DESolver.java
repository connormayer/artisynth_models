package artisynth.models.fluid1d;

import maspack.matrix.*;

public abstract class DESolver 
{
    int N;		// number of points/eqns

    public enum DDXIntScheme {ddx_fore, ddx_fore2, ddx_fore3, ddx_fore4, ddx_fore5, ddx_back, ddx_back2, ddx_back3, ddx_back4, ddx_back5, ddx_cent, ddx_cent5, ddx_foreMix4, ddx_backMix4};
    public enum DDTIntScheme {ddt_back, ddt_back2, ddt_cent, ddt_backMix4, ignore};

    public enum Preconditioner {none, jacobi, scaling};
    public Preconditioner preconditioner = Preconditioner.none;

    public enum SolveType {direct, normalEqns};
    public SolveType solveType = SolveType.normalEqns;

    public abstract void setNumberOfPoints(int n);

    public abstract void clear();

    public abstract void addToB(int iR, double value);

    public abstract void addToB(double[] values);

    public abstract void addToA(int i, int j, double val);

    public abstract void setA(int i, int j, double val);
    public abstract double getA(int i, int j);
    public abstract void setB(int i, double val);
    public abstract double getB(int i);

    //   public abstract void scalarTerm(int iR, int iC, double scalar);
    //   public abstract void ddxTerm(int iR, int iC, double scalar, DDXIntScheme scheme, double dx);
    //   public abstract void ddtTerm(int iR, int iC, double scalar, DDTIntScheme scheme, double dt, double x0, double x00);

    public abstract void solve();

    public abstract double[] getSolution();

    public abstract void writeA(String filename);
    public abstract void writeb(String filename);
    public abstract double computeResidual();

    // Implementations

    public void addMatrixDamping_const(double lambda)
    {
        for (int i=0; i<N; i++)
            addToA(i,i, lambda);
    }

    public void addMatrixDamping_scaled(double lambda)
    {
        for (int i=0; i<N; i++)
            addToA(i,i, lambda*getA(i,i));
    }

    public void scaleSystem()
    {
        for (int i=0; i<N; i++)
        {
            double max = 0.0;
            for (int j=0; j<N; j++)
            {
                if (Math.abs( getA(i,j) ) > max)
                    max = Math.abs( getA(i,j) );
            }
            for (int j=0; j<N; j++)
            {
                setA(i,j, getA(i,j)/max);
            }
            setB(i, getB(i)/max);
        }
    }

    public void precondition_jacobi()
    {
        for (int i=0; i<N; i++)
        {
            double Aii = getA(i,i);
            if (Math.abs(Aii) < 0.00000001)
                System.out.println("matrix warning!!");
            else
            {
                setA(i,i, Aii/Aii);//heh...
                setB(i,   getB(i)/Aii);
            }
        }
    }

    public void scalarTerm(int iR, int iC, double scalar)
    {
        addToA(iR,iC, scalar);
    }

    public void ddxTerm(int iR, int iC, double scalar, DDXIntScheme scheme, double dx)
    {

        if      (scheme == DDXIntScheme.ddx_fore)
        {
            addToA(iR,iC+1,  1.0*scalar/dx);
            addToA(iR,iC,   -1.0*scalar/dx);
        }
        else if (scheme == DDXIntScheme.ddx_fore2)
        {
            addToA(iR,iC,   -3.0*scalar/(2.0*dx));
            addToA(iR,iC+1, +4.0*scalar/(2.0*dx));
            addToA(iR,iC+2, -1.0*scalar/(2.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_fore3)
        {
            addToA(iR,iC,   -11.0*scalar/(6.0*dx));
            addToA(iR,iC+1, +18.0*scalar/(6.0*dx));
            addToA(iR,iC+2, - 9.0*scalar/(6.0*dx));
            addToA(iR,iC+3, + 2.0*scalar/(6.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_fore4)
        {
            addToA(iR,iC,   -25.0*scalar/(12.0*dx));
            addToA(iR,iC+1, +48.0*scalar/(12.0*dx));
            addToA(iR,iC+2, -36.0*scalar/(12.0*dx));
            addToA(iR,iC+3, +16.0*scalar/(12.0*dx));
            addToA(iR,iC+4, - 3.0*scalar/(12.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_fore5)
        {
            addToA(iR,iC,   -137.0*scalar/(60.0*dx));
            addToA(iR,iC+1, +300.0*scalar/(60.0*dx));
            addToA(iR,iC+2, -300.0*scalar/(60.0*dx));
            addToA(iR,iC+3, +200.0*scalar/(60.0*dx));
            addToA(iR,iC+4, - 75.0*scalar/(60.0*dx));
            addToA(iR,iC+5, + 12.0*scalar/(60.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_back)
        {
            addToA(iR,iC,    1.0*scalar/dx);
            addToA(iR,iC-1, -1.0*scalar/dx);
        }
        else if (scheme == DDXIntScheme.ddx_back2)
        {
            addToA(iR,iC,   +3.0*scalar/(2.0*dx));
            addToA(iR,iC-1, -4.0*scalar/(2.0*dx));
            addToA(iR,iC-2, +1.0*scalar/(2.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_back3)
        {
            addToA(iR,iC,   +11.0*scalar/(6.0*dx));
            addToA(iR,iC-1, -18.0*scalar/(6.0*dx));
            addToA(iR,iC-2, + 9.0*scalar/(6.0*dx));
            addToA(iR,iC-3, - 2.0*scalar/(6.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_back4)
        {
            addToA(iR,iC,   +25.0*scalar/(12.0*dx));
            addToA(iR,iC-1, -48.0*scalar/(12.0*dx));
            addToA(iR,iC-2, +36.0*scalar/(12.0*dx));
            addToA(iR,iC-3, -16.0*scalar/(12.0*dx));
            addToA(iR,iC-4, + 3.0*scalar/(12.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_back5)
        {
            addToA(iR,iC,   +137.0*scalar/(60.0*dx));
            addToA(iR,iC-1, -300.0*scalar/(60.0*dx));
            addToA(iR,iC-2, +300.0*scalar/(60.0*dx));
            addToA(iR,iC-3, -200.0*scalar/(60.0*dx));
            addToA(iR,iC-4, + 75.0*scalar/(60.0*dx));
            addToA(iR,iC-5, - 12.0*scalar/(60.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_foreMix4)
        {
            addToA(iR,iC-1, -2.0*scalar/(6.0*dx));
            addToA(iR,iC,   -3.0*scalar/(6.0*dx));
            addToA(iR,iC+1, +6.0*scalar/(6.0*dx));
            addToA(iR,iC+2, -1.0*scalar/(6.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_backMix4)
        {
            addToA(iR,iC+1, +2.0*scalar/(6.0*dx));
            addToA(iR,iC,   +3.0*scalar/(6.0*dx));
            addToA(iR,iC-1, -6.0*scalar/(6.0*dx));
            addToA(iR,iC-2, +1.0*scalar/(6.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_cent)
        {
            addToA(iR,iC+1,  1.0*scalar/(2.0*dx));
            addToA(iR,iC-1, -1.0*scalar/(2.0*dx));
        }
        else if (scheme == DDXIntScheme.ddx_cent5)
        {
            addToA(iR,iC+2, -1.0*scalar/(12.0*dx));
            addToA(iR,iC+1,  8.0*scalar/(12.0*dx));
            addToA(iR,iC-1, -8.0*scalar/(12.0*dx));
            addToA(iR,iC-2,  1.0*scalar/(12.0*dx));
        }
        else
        {
            System.out.println("DE Solver: Invalid ddx scheme");
        }

    }

    public void ddtTermUsingRHS(int iR, int iC, double scalar, DDTIntScheme scheme, double dt, double x0, double x00)
    {

        if      (scheme == DDTIntScheme.ddt_back)
        {
            addToA(iR,iC,  1.0*scalar/dt);
            addToB(iR,   1.0*x0*scalar/dt);  		//b.add(iR, 0,    1.0*scalar/dt);	// adding to rhs
        }
        else if (scheme == DDTIntScheme.ddt_back2)
        {
            addToA(iR,iC,  3.0*scalar/(2.0*dt));
            addToB(iR,   (4.0*x0 - x00)*scalar/(2.0*dt)); //b.add(iR, 0,    1.0*scalar/dt);	// adding to rhs
        }
        else if (scheme == DDTIntScheme.ddt_cent)
        {
            addToA(iR,iC,  1.0*scalar/(2.0*dt));
            addToB(iR,   1.0*x00*scalar/(2.0*dt));	//b.add(iR, 0,    1.0*scalar/dt);	// adding to rhs
        }
        else if (scheme == DDTIntScheme.ignore)
        {
            // acknowledged error
        }
        else
        {
            System.out.println("DE Solver: Invalid ddt scheme");
        }

    }

    /*//
   public void ddtTerm(int iR, int iC, int iT0, int iT00, double scalar, DDTIntScheme scheme, double dt)
   {

      if      (scheme == DDTIntScheme.ddt_back)
      {
	 addToA(iR,iC,  +1.0*scalar/dt);
	 addToA(iR,iT0, -1.0*scalar/dt);
      }
      else if (scheme == DDTIntScheme.ddt_back2)
      {
	 addToA(iR,iC,   +3.0*scalar/(2.0*dt));
	 addToA(iR,iT0,  -4.0*scalar/(2.0*dt));
	 addToA(iR,iT00, +1.0*scalar/(2.0*dt));
      }
      else if (scheme == DDTIntScheme.ddt_backMix4)
      {
	 // TODO: untested!
	 addToA(iR,iC,    +2.0*scalar/(6.0*dt));
	 addToA(iR,iT0,   +3.0*scalar/(6.0*dt));
	 addToA(iR,iT00,  -6.0*scalar/(6.0*dt));
	 addToA(iR,iT000, +1.0*scalar/(6.0*dt));
      }
      else if (scheme == DDTIntScheme.ddt_cent)
      {
	 addToA(iR,iC,   +1.0*scalar/(2.0*dt));
	 addToA(iR,iT00, -1.0*scalar/(2.0*dt));
      }
      else if (scheme == DDTIntScheme.ignore)
      {
	 // acknowledged error
      }
      else
      {
	 System.out.println("DE Solver: Invalid ddt scheme");
      }

   }
//*/

    public void ddtTerm(int iR, int iC, int iT0, int iT00, int iT000, double scalar, DDTIntScheme scheme, double dt)
    {

        if      (scheme == DDTIntScheme.ddt_back)
        {
            addToA(iR,iC,  +1.0*scalar/dt);
            addToA(iR,iT0, -1.0*scalar/dt);
        }
        else if (scheme == DDTIntScheme.ddt_back2)
        {
            addToA(iR,iC,   +3.0*scalar/(2.0*dt));
            addToA(iR,iT0,  -4.0*scalar/(2.0*dt));
            addToA(iR,iT00, +1.0*scalar/(2.0*dt));
        }
        else if (scheme == DDTIntScheme.ddt_backMix4)
        {
            // TODO: untested!
            addToA(iR,iC,    +2.0*scalar/(6.0*dt));
            addToA(iR,iT0,   +3.0*scalar/(6.0*dt));
            addToA(iR,iT00,  -6.0*scalar/(6.0*dt));
            addToA(iR,iT000, +1.0*scalar/(6.0*dt));
        }
        else if (scheme == DDTIntScheme.ddt_cent)
        {
            addToA(iR,iC,   +1.0*scalar/(2.0*dt));
            addToA(iR,iT00, -1.0*scalar/(2.0*dt));
        }
        else if (scheme == DDTIntScheme.ignore)
        {
            // acknowledged error
        }
        else
        {
            System.out.println("DE Solver: Invalid ddt scheme");
        }

    }

}
