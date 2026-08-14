package artisynth.models.frank2.frankUtilities;

import java.util.ArrayList;
import maspack.matrix.*;
import maspack.solvers.PardisoSolver;

public class RadialBasisFunction 
{
    ArrayList<Point3d> x;		// known data points
    double[] fx;                // the known function

    // MatrixNd phi;
    MatrixNd lambda;			// the interpolation coefficients

    int dim = 3;                // only support 3d right now
    
    public enum Kernal {multiquadric, inverse, gaussian, linear, cubic, quintic, thin_plate};
    Kernal kernal = Kernal.multiquadric;

    
    public RadialBasisFunction(ArrayList<Point3d> points, double[] values, Kernal kernal)
    {
        Initialize(points, values, kernal);
    }
    
    public RadialBasisFunction(ArrayList<Point3d> points, double[] values)
    {
        Initialize(points, values, kernal);
    }
    
    void Initialize(ArrayList<Point3d> points, double[] values, Kernal kernal)
    {
        x = points;
        fx = values;
        this.kernal = kernal;
        UpdateLambda();
    }

    public void UpdateLambda()
    {
        // lambda = phi^-1 * f
        
        MatrixNd phi = BuildPhi(x,x);
        MatrixNd f = BuildF(fx);
        lambda = new MatrixNd(x.size()+dim+1, 1);
        
        /*
        LUDecomposition chd = new LUDecomposition(phi);
        chd.solve(lambda, f);
        
        //phi.invert(); // only OK for small problems
        //lambda.mul(phi, f);
        /*/
        // this is a dense matrix, but Pardiso is still 2x a fast for a large problem
        PardisoSolver pardiso = new PardisoSolver();
        pardiso.analyze(phi, phi.rowSize(), Matrix.INDEFINITE);
        pardiso.factor();
        pardiso.solve(lambda.getBuffer(), f.getBuffer());
        //*/

    }

    public double[] Interpolate(ArrayList<Point3d> y)
    {
        
        MatrixNd phi = BuildPhi(x,y);
        MatrixNd Fy = new MatrixNd();
        Fy.mul(phi, lambda);
        
        double[] fy = new double[y.size()];
        for (int a=0; a<fy.length; a++)
        {
            fy[a] = Fy.get(a, 0);
        }
        
        return fy;
    }

    public MatrixNd BuildPhi(ArrayList<Point3d> x, ArrayList<Point3d> y)
    {
        // x is the array of known data points
        // y is the array of unknown data points

        int nCols = x.size() + dim + 1;
        int nRows = y.size() + dim + 1;
        MatrixNd phi = new MatrixNd(nRows, nCols);

        // calculate epsilon
        int count = 0;
        double epsilon = 0.0;
        for (int i=0; i<x.size(); i++)
        {
            for (int j=0; j<y.size(); j++)
            {
                count++;
                epsilon = epsilon + x.get(i).distance(y.get(j));
            }
        }
        epsilon = epsilon/((double)count);
        
        for (int i=0; i<x.size(); i++)
        {
            for (int j=0; j<y.size(); j++)
            {
                double r = x.get(i).distance(y.get(j));
                double fxy = 0.0;
                
                if      (kernal == Kernal.multiquadric)
                    fxy = Math.sqrt(Math.pow(r/epsilon, 2.0) + 1.0);
                else if (kernal == Kernal.inverse)
                    fxy = 1.0/Math.sqrt(Math.pow(r/epsilon, 2.0) + 1.0);
                else if (kernal == Kernal.gaussian)
                    fxy = Math.exp(Math.pow(-r/epsilon, 2.0));
                else if (kernal == Kernal.linear)
                    fxy = r;
                else if (kernal == Kernal.cubic)
                    fxy = r*r*r;
                else if (kernal == Kernal.quintic)
                    fxy = r*r*r*r*r;
                else if (kernal == Kernal.thin_plate)
                    fxy = r*r*Math.log(r);
                
                //phi.set(j, i, Kernal(x.get(i), y.get(j)) );
                phi.set(j, i, fxy);
            }
        }

        for (int i=0; i<x.size(); i++)
        {
            int j = y.size();
            phi.set(j, i, 1.0);
            for (int a=0; a<dim; a++)
            {
                phi.set(j+a+1, i, x.get(i).get(a));
            }
        }

        for (int j=0; j<y.size(); j++)
        {
            int i = x.size();
            phi.set(j, i, 1.0);
            for (int a=0; a<dim; a++)
            {
                phi.set(j, i+a+1, y.get(j).get(a));
            }
        }

        return phi;
    }

    /*//
    public double Kernal(Point3d x1, Point3d x2)
    {   
//        'multiquadric': sqrt((r/self.epsilon)**2 + 1)
//        'inverse': 1.0/sqrt((r/self.epsilon)**2 + 1)
//        'gaussian': exp(-(r/self.epsilon)**2)
//        'linear': r
//        'cubic': r**3
//        'quintic': r**5
//        'thin_plate': r**2 * log(r)
        
        double r = x1.distance(x2);
        double basis = 0.0;
        
        if      (kernal == Kernal.multiquadric)
            Math.sqrt(Math.pow(r/ep, 2.0) + 1.0);
        else if (kernal == Kernal.inverse)
            ;
        else if (kernal == Kernal.gaussian)
            ;
        else if (kernal == Kernal.linear)
            basis = r;
        else if (kernal == Kernal.cubic)
            basis = r*r*r;
        else if (kernal == Kernal.quintic)
            basis = r*r*r*r*r;
        else if (kernal == Kernal.thin_plate)
            basis = r*r*Math.log(r);

        return basis;    
    }
    //*/

    public MatrixNd BuildF(double[] f)
    {
        int nRows = f.length + dim + 1;
        MatrixNd F = new MatrixNd(nRows, 1);

        for (int a=0; a<f.length; a++)
        {
            F.set(a, 0, f[a]);
        }

        for (int a=f.length; a<nRows; a++)
        {
            F.set(a, 0, 0.0);		// the remaining elements are 0.0
        }

        return F;
    }

}
