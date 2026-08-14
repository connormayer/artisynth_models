package artisynth.models.fluid1d.fluidUtils;

public class ArrayMath 
{
   
   public static double[] add(double[] arr1, double[] arr2)
   {
      if (arr1.length == arr2.length)
      {
	 int N = arr1.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = arr1[n] + arr2[n];
	 return res;
      }
      else
	 return null;
   }
   
   public static double[] addScalar(double[] arr1, double scalar)
   {
      int N = arr1.length;
      double[] res = new double[N];
      for (int n=0; n<N; n++)
	 res[n] = arr1[n] + scalar;
      return res;
      
   }
   
   public static double[] sub(double[] arr2, double[] arr1)
   {
      if (arr1.length == arr2.length)
      {
	 int N = arr1.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = arr2[n] - arr1[n];
	 return res;
      }
      else
	 return null;
   }
   
   public static double[] mult(double[] arr1, double[] arr2)
   {
      if (arr1.length == arr2.length)
      {
	 int N = arr1.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = arr1[n] * arr2[n];
	 return res;
      }
      else
	 return null;
   }
   
   public static double[] mult(double scalar, double[] arr)
   {
      int N = arr.length;
      double[] res = new double[N];
      for (int n=0; n<N; n++)
	 res[n] = arr[n] * scalar;
      return res;
   }
   
   public static double[] mult(double[] arr, double scalar)
   {
      int N = arr.length;
      double[] res = new double[N];
      for (int n=0; n<N; n++)
	 res[n] = arr[n] * scalar;
      return res;
   }
   
   public static double[] div(double[] arr1, double[] arr2)
   {
      if (arr1.length == arr2.length)
      {
	 int N = arr1.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = arr1[n] / arr2[n];
	 return res;
      }
      else
	 return null;
   }
   
   public static double[] div(double[] arr1, double scalar)
   {
      int N = arr1.length;
      double[] res = new double[N];
      for (int n=0; n<N; n++)
	 res[n] = arr1[n] / scalar;
      return res;
   }
   
   public static double[] div(double scalar, double[] arr1)
   {
      int N = arr1.length;
      double[] res = new double[N];
      for (int n=0; n<N; n++)
	 res[n] = scalar/arr1[n];
      return res;
   }
   
   public static double[] scale(double scale, double[] arr)
   {
      int N = arr.length;
      double[] res = new double[N];
      for (int n=0; n<N; n++)
	 res[n] = scale*arr[n];
      
      return res;
   }
   
   public static double[] abs(double[] arr)
   {
      int N = arr.length;
      double[] res = new double[N];
      for (int a=0; a<arr.length; a++)
	 res[a] = Math.abs(arr[a]);
      
      return res;
   }
   
   public static double norm(double[] arr)
   {
      double n = 0.0;
      for (int a=0; a<arr.length; a++)
	 n = n + arr[a]*arr[a];
      n = Math.sqrt(n);
      
      return n;
   }
   
   public static double dot(double[] arr1, double[] arr2)
   {
      if (arr1.length == arr2.length)
      {
	 int N = arr1.length;
	 double dot = 0.0;
	 for (int n=0; n<N; n++)
	    dot = dot + arr1[n]*arr2[n];
	 return dot;
      }
      else
	 return Double.NaN;
   }
   
   public static double[] ddx(double[] arr, double dx)
   {
      return ddx_cent(arr,dx);
   }
   
   public static double[] ddx_fore(double[] arr, double dx)
   {
      // 1st order foreward diff
      int N = arr.length;
      double[] res = new double[N];
      for (int n=0; n<N-1; n++)
	 res[n] = (arr[n+1] - arr[n])/dx;
      res[N-1] = (arr[N-1] - arr[N-2])/dx;	// so, res[N-1] = res[N-2]
      
      return res;      
   }
   
   public static double[] ddx_back(double[] arr, double dx)
   {
      // 1st order backward diff
      int N = arr.length;
      double[] res = new double[N];
      for (int n=1; n<N; n++)
	 res[n] = (arr[n] - arr[n-1])/dx;
      res[0] = (arr[1] - arr[0])/dx;	// so, res[0] = res[1]
      
      return res;      
   }
   
   public static double[] ddx_cent(double[] arr, double dx)
   {
      // 2nd order central differencing
      int N = arr.length;
      double[] res = new double[N];
      for (int n=1; n<N-1; n++)
	 res[n] = (arr[n+1] - arr[n-1])/(2.0*dx);
      
      res[0] =   (-3.0*arr[0]   + 4.0*arr[1]   - 1.0*arr[2])  /(2.0*dx);	// 2nd order foreward
      res[N-1] = ( 3.0*arr[N-1] - 4.0*arr[N-2] + 1.0*arr[N-3])/(2.0*dx);	// 2nd order backward
      
      return res;      
   }
   
   public static double[] ddt(double[] arr1, double[] arr0, double dt)
   {
      // 1st order time diff
      if (arr0.length == arr1.length)
      {
	 int N = arr0.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = (arr1[n] - arr0[n])/dt;
	 
	 return res;
      }
      else
      {
	 return null;
      }
   }
   
   public static double[] ddt_back2(double[] arr2, double[] arr1, double[] arr0, double dt)
   {
      // calculates the 2nd order accurate backward derivative across the arrays (so the derivative is at time "2")
      if ( (arr0.length == arr1.length) && (arr0.length == arr2.length) )
      {
	 int N = arr0.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = (3.0*arr2[n] -4.0*arr1[n] + 1.0*arr0[n])/(2.0*dt);
	 
	 return res;
      }
      else
      {
	 return null;
      }
   }
   
   public static double[] ddt_back3(double[] arr3, double[] arr2, double[] arr1, double[] arr0, double dt)
   {
      // calculates the 2nd order accurate backward derivative across the arrays (so the derivative is at time "2")
      if ( (arr0.length == arr1.length) && (arr0.length == arr2.length) && (arr0.length == arr3.length) )
      {
	 int N = arr0.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = (arr3[n] + arr2[n] - arr1[n] - arr0[n])/(4.0*dt);
	 
	 return res;
      }
      else
      {
	 return null;
      }
   }
   
   public static double[] ddt_cent(double[] arr2, double[] arr0, double dt)
   {
      // calculates the 2nd order accurate central derivative across the arrays (so the derivative is at time "1")
      // warning: using this for time advance means that the ddt terms will actually be 1 time step behind the ddx terms
      if (arr0.length == arr2.length)
      {
	 int N = arr0.length;
	 double[] res = new double[N];
	 for (int n=0; n<N; n++)
	    res[n] = (1.0*arr2[n] - 1.0*arr0[n])/(2.0*dt);
	 
	 return res;
      }
      else
      {
	 return null;
      }
   }
   
   public static double[] integrate(double[] x, double dx)
   {
      int L = x.length;
      double[] I = new double[L];
      
      I[0] = 0.0;
      for (int a=1; a<L; a++)
      {
	 I[a] = I[a-1] + ((x[a] + x[a-1])/2.0)*dx;
      }
      return I;
   }
   
   public static double[] deepCopy_doubleArray(double[] arr)
   {
      double[] copy = new double[arr.length];
      for (int a=0; a< arr.length; a++)
	 copy[a] = arr[a];
      
      return copy;
   }

}
