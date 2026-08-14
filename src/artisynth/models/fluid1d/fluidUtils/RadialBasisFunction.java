package artisynth.models.fluid1d.fluidUtils;

import java.util.ArrayList;
import maspack.matrix.*;
import maspack.solvers.PardisoSolver;

public class RadialBasisFunction 
{
   ArrayList<Point3d> x;		// known data points
   ArrayList<Point3d> y;		// unknown data points
   double[] fx;				// the known function
   double[] fy;				// interpolated values
   
   // MatrixNd phi;
   MatrixNd lambda;			// the interpolation coefficients
   
   int dim;
   
   public void Update()
   {
      // this can be called once x,y, and fx are set
      UpdateLambda();			// find the coefficients (lambda) --> only needed if x or f changes
      UpdateInterpolation();      	// find the interpolation values
   }
   
   public void UpdateLambda()
   {
      // lambda = phi^-1 * f
      MatrixNd phi = BuildPhi(x,x);
      MatrixNd f = BuildF(fx);
      
      phi.invert();
      lambda.mul(phi, f);
      
//      PardisoSolver pardiso = new PardisoSolver();
//      pardiso.factor(phi);
//      double[] lambdaArr = new double[lambda.rowSize()];
//      double[] fArr      = new double[f.rowSize()];
//      f.get(fArr);
//      pardiso.solve(lambdaArr,fArr);
//      lambda.set(lambdaArr);
   }
   
   public void UpdateInterpolation()
   {
      MatrixNd phi = BuildPhi(x,y);
      MatrixNd Fy = new MatrixNd();
      
      Fy.mul(phi, lambda);
      fy = ExtractF(Fy);
   }
   
   public void SetKnownPoints(ArrayList<Point3d> points)
   {
      x = points;
      
      // this allows some initializations
      dim = 3;						// could be generalized
      lambda = new MatrixNd(x.size()+dim+1, 1);		// should I set the size here??
   }
   
   public void SetKnownValues(double[] f)
   {
      fx = f;
   }
   
   public void SetInterpolationPoints(ArrayList<Point3d> points)
   {
      y = points;
      fy = new double[y.size()];
   }
   
   public double[] GetInterpolationValues()
   {
      return fy;
   }
   
   public MatrixNd BuildPhi(ArrayList<Point3d> x, ArrayList<Point3d> y)
   {
      // x is the array of known data points
      // y is the array of unknown data points
      
      int nCols = x.size() + dim + 1;
      int nRows = y.size() + dim + 1;
      MatrixNd phi = new MatrixNd(nRows, nCols);
      
      for (int i=0; i<x.size(); i++)
      {
	 for (int j=0; j<y.size(); j++)
	 {
	    phi.set(j, i, Kernal(x.get(i), y.get(j)) );
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
   
   public double Kernal(Point3d x1, Point3d x2)
   {
      // there are certainly other rbf types to use here
      
      double r = x1.distance(x2);
      double basis = 0.0;
      
      if (dim == 3)
	 basis = r*r*r;
      else if(dim == 2)
	 basis = r*r*Math.log(r);
      else if (dim == 1)
	 basis = r;
      
      return basis;    
   }
   
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
   
   public double[] ExtractF(MatrixNd F)
   {
      double[] f = new double[F.rowSize()-dim-1];
      
      for (int a=0; a<f.length; a++)
      {
	 f[a] = F.get(a, 0);
      }
      
      return f;
   }

}
