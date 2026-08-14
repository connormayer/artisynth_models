package artisynth.models.fluid1d;

import java.io.*;

import maspack.matrix.*;
import maspack.solvers.PardisoSolver;

public class DESolver_Sparse extends DESolver
{
    double[] x;			// the solution
    double[] b;			// the right hand side
    SparseMatrixNd A;		// the NxN matrix
    PardisoSolver pardiso = new PardisoSolver();

    //   SparseMatrixNd AT;
    //   SparseMatrixNd ATA;
    //   SparseMatrixNd ATb;


    public void setNumberOfPoints(int n)
    {
        N = n;
        x = new double[N];
        b = new double[N];
        A = new SparseMatrixNd(N,N);

        // only if using normal eqns...
        //      AT  = new SparseMatrixNd(N,N);
        //      ATA = new SparseMatrixNd(N,N);
        //      ATb = new SparseMatrixNd(N,1);
    }

    public void clear()
    {
        // reset the A matrix to all zeros
        A.setZero();
        for (int n=0; n<N; n++)
        {
            b[n] = 0.0;
            A.set(n,n,0.0);	//pardiso might require diagonals defined...even if zero.
        }
    }

    public void addToA(int i, int j, double val)
    {
        if (val != 0.0)
            A.set(i,j, A.get(i,j) + val);
    }

    public void addToB(int iR, double value)
    {
        b[iR] = b[iR] + value;
    }

    public void addToB(double[] values)
    {
        //TODO: just replace b??
        for (int a=0; a< values.length; a++)
            addToB(a, values[a]);
    }

    public void setA(int i, int j, double val) 
    {
        if (val != 0.0)
            A.set(i,j, val);	// be careful not to add zeros to the sparse matrix
    }

    public double getA(int i, int j) 
    {
        return A.get(i,j);
    }

    public void setB(int i, double val) 
    {
        b[i] = val;
    }

    public double getB(int i) 
    {
        return b[i];
    }


    public void solve()
    {
        if       (preconditioner == Preconditioner.none)
        {
        }
        else if (preconditioner == Preconditioner.scaling)
        {
            scaleSystem();		// not really a preconditioner, but for now...
        }
        else if (preconditioner == Preconditioner.jacobi)
        {
            precondition_jacobi();
        }
        else
            System.err.println("Sparse Solver: preconditioner not implemented.");

        if      (solveType == SolveType.direct)
        {
            //System.out.printf("factor... ");
            pardiso.analyzeAndFactor(A);
            //System.out.printf("solve... ");
            pardiso.solve(x, b);
        }
        else if (solveType == SolveType.normalEqns)
        {
            // solve the normal equations
            scaleSystem();

            SparseMatrixNd AT = new SparseMatrixNd(A);
            SparseMatrixNd ATA = new SparseMatrixNd(N,N);
            SparseMatrixNd ATb = new SparseMatrixNd(N,1);

            AT.transpose();
            ATA.mul(AT,A);
            ATb.mul(AT, new SparseMatrixNd(new MatrixNd(N,1,b)));

            /*// Cholesky is slower than Pardiso (factor of 2?)
            MatrixNd ata = new MatrixNd(ATA);
            CholeskyDecomposition chd = new CholeskyDecomposition(ata);
            MatrixNd xM = new MatrixNd(N,1);
            chd.solve(xM,ATb);
            xM.get(x);
            //*/

            //*// Pardiso solve
            // TODO: this code used to work, but is now the source of a massive memory leak
            //pardiso.analyzeAndFactor(ATA); // does LU factor
            //pardiso.analyze(ATA, ATA.rowSize(), Matrix.SPD);
            pardiso.analyze(ATA, ATA.rowSize(), Matrix.SYMMETRIC);
            pardiso.factor();
            ATb.get(b);
            pardiso.solve(x, b);
            //*/
        }

        //System.out.printf("residPar=%e, ", computeResidual());
    }

    public double computeResidual()
    {  // A*x-b = 0 ...in the ideal world --> better yet: ||A x - b || / || b ||
        VectorNd rV = new VectorNd(x.length);
        VectorNd xV = new VectorNd(x.length);
        VectorNd bV = new VectorNd(x.length);
        xV.set(x);
        bV.set(b);
        rV.mul (A, xV);
        rV.sub (bV);
        double res = rV.norm();

        return res;
    }

    public double[] getSolution()
    {
        return x;
    }

    public void writeA(String filename)
    {
        //      double[][] matrix = new double[A.rowSize()][A.colSize()];
        //      A.get(matrix);
        //      artisynth.models.peterUtilities.fileIO.CSV.Write(filename, matrix);

        try
        {
            A.write (new PrintWriter(new BufferedWriter(new FileWriter(filename, false))), new maspack.util.NumberFormat("%g"), maspack.matrix.Matrix.WriteFormat.CRS);
        }
        catch(Exception e)
        {
        }

    }

    public void writeb(String filename)
    {
        //double[][] matrix = new double[A.rowSize()][A.colSize()];
        //A.get(matrix);
        artisynth.models.fluid1d.fileIO.CSV.WriteVector(filename, b);
    }


}
