package artisynth.models.fluid1d;

import maspack.matrix.*;
//import maspack.solvers.PardisoSolver;
import maspack.solvers.PardisoSolver;

public class DESolver_Dense extends DESolver
{
    MatrixNd x;		// the solution
    MatrixNd b;		// the right hand side
    MatrixNd A;		// the NxN matrix

    // if forming normal equations
    //   MatrixNd AT;
    //   MatrixNd ATA;
    //   MatrixNd ATb;


    public void setNumberOfPoints(int n)
    {
        this.N = n;
        x = new MatrixNd(N,1);
        b = new MatrixNd(N,1);
        A = new MatrixNd(N,N);

        //      AT  = new MatrixNd(N,N);
        //      ATA = new MatrixNd(N,N);
        //      ATb = new MatrixNd(N,1);
    }

    public void clear()
    {
        // reset the A matrix to all zeros
        A.setZero();
        b.setZero();
    }

    public void addToA(int i, int j, double val) 
    {
        A.add(i,j, val);
    }

    public void addToB(int iR, double value)
    {
        b.add(iR, 0, value);
    }

    public void addToB(double[] values)
    {
        for (int a=0; a< values.length; a++)
            addToB(a, values[a]);
    }

    public void setA(int i, int j, double val) 
    {
        A.set(i,j, val);      
    }

    public double getA(int i, int j) 
    {
        return A.get(i,j);
    }

    public void setB(int i, double val) 
    {
        b.set(i,0, val);
    }

    public double getB(int i) 
    {
        return b.get(i,0);
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

        if (solveType == SolveType.direct)
        {
            // the naive approach (the inversion uses LU) --> slow, but accurate (matrix resid is 0)
            //MatrixNd Ainv = new MatrixNd();
            //Ainv.invert(A);
            //x.mul(Ainv,b);
            //double condNum = Ainv.frobeniusNorm()*A.frobeniusNorm();	// condition number
            //System.out.printf("cond num = %f \t", condNum);

            // LU Decomposition --> seems to be the same speed as above
            LUDecomposition lud = new LUDecomposition(A);
            lud.solve(x ,b);
        }
        else if (solveType == SolveType.normalEqns)
        {
            // Cholesky Decomposition --> significantly slower than LU

            MatrixNd AT = new MatrixNd(A);
            MatrixNd ATA = new MatrixNd();
            MatrixNd ATb = new MatrixNd();

            scaleSystem();	// I must scale the system before creating normal eqns, otherwise the condition number explodes.

            //System.out.println("transposing...");
            AT.transpose(A);
            //System.out.println("matrix mul...");
            ATA.mul(AT, A);	// this is the slow step
            //System.out.println("vector mul...");
            ATb.mul(AT,b);
            //System.out.println("solving...");

            //*/
            // cholesky beats pardiso using dense, normal eqns
            CholeskyDecomposition chd = new CholeskyDecomposition(ATA);
            chd.solve(x,ATb);
            //*/

            /*/
	 // Pardiso solve --> 
	 double[] xArr = new double[x.rowSize()];
	 x.get(xArr);
	 double[] bArr = new double[b.rowSize()];
	 ATb.get(bArr);

	 PardisoSolver pardiso = new PardisoSolver();
	 pardiso.analyzeAndFactor(ATA);
	 pardiso.solve(xArr, bArr);
	 x.set(xArr);
	 //*/
        }




    }

    public double computeResidual() 
    {
        MatrixNd r = new MatrixNd (A.rowSize(),1);
        r.mul (A, x);
        r.sub (b);
        double res = r.frobeniusNorm();

        //System.out.println("residual = " + res);
        return res;
    }

    public double[] getSolution()
    {
        double[] uArr = new double[x.rowSize()];
        x.get(uArr);
        return uArr;
    }

    public void writeA(String filename)
    {
        double[][] matrix = new double[A.rowSize()][A.colSize()];
        A.get(matrix);
        artisynth.models.fluid1d.fileIO.CSV.Write(filename, matrix);
    }

    public void writeb(String filename)
    {
        double[][] matrix = new double[b.rowSize()][b.colSize()];
        b.get(matrix);
        artisynth.models.fluid1d.fileIO.CSV.Write(filename, matrix);
    }   

}
