package artisynth.models.fluid1d.fluidUtils;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

import artisynth.core.femmodels.*;
import artisynth.core.mechmodels.RigidBody;

public class GeometryCreator
{
   
   public static int deltaToN(double lenX, double dx)
   {
      // find a safe integer number of intervals...maybe not the best approach here
      return Math.round((float)Math.ceil(lenX/dx));
   }
   
   // --- FEMs --- //
   
   public static FemModel3d createFEMBox(double[] pMin, double[] pMax, double dx, double dy, double dz)
   {
      double xLen = pMax[0] - pMin[0];
      double yLen = pMax[1] - pMin[1];
      double zLen = pMax[2] - pMin[2];
      double posX = (pMax[0] + pMin[0])/2.0;
      double posY = (pMax[1] + pMin[1])/2.0;
      double posZ = (pMax[2] + pMin[2])/2.0;
      return createFEMBox(xLen, yLen, zLen, dx, dy, dz, posX, posY, posZ);
   }
   
   public static FemModel3d createFEMBox(double lenX, double lenY, double lenZ, double dx, double dy, double dz, double posX, double posY, double posZ)
   {
      return createFEMBox(lenX,lenY,lenZ, deltaToN(lenX,dx),deltaToN(lenY,dy),deltaToN(lenZ,dz), posX, posY, posZ);
   }
   
   public static FemModel3d createFEMBox(double lenX, double lenY, double lenZ, int nX, int nY, int nZ, double posX, double posY, double posZ)
   {
      FemModel3d fem = new FemModel3d();
      FemFactory.createHexGrid(fem, lenX, lenY, lenZ, nX, nY, nZ);
      fem.setName("femBox");
      
      fem.transformGeometry (new RigidTransform3d ( new Vector3d (posX, posY, posZ), new AxisAngle(1.0, 0.0, 0.0, 0.0*Math.PI/180.0) ));
      
      return fem;
   }
   
   public static FemModel3d createFEMTubeTet(double lenX, double rIn, double rOut, int nX, int nThick, int nCirc, double posX, double posY, double posZ)
   {
      // creates a tube oriented in along the x-axis...
      FemModel3d fem = new FemModel3d();
      FemFactory.createTetTube(fem, lenX, rIn, rOut, nCirc, nX, nThick);
      fem.setName("femTube");
      fem.transformGeometry (new RigidTransform3d ( new Vector3d (), new AxisAngle(0.0, 1.0, 0.0, 90.0*Math.PI/180.0) ));
      fem.transformGeometry (new RigidTransform3d ( new Vector3d (posX, posY, posZ), new AxisAngle(1.0, 0.0, 0.0, 0.0*Math.PI/180.0) ));

      return fem;
   }
   
   public static FemModel3d createFEMTube(double lenX, double rIn, double rOut, int nX, int nThick, int nCirc, double posX, double posY, double posZ)
   {
      // creates a tube oriented in along the x-axis...
      FemModel3d fem = new FemModel3d();
      FemFactory.createHexTube(fem, lenX, rIn, rOut, nCirc, nX, nThick);
      fem.setName("femTube");
      fem.transformGeometry (new RigidTransform3d ( new Vector3d (), new AxisAngle(0.0, 1.0, 0.0, 90.0*Math.PI/180.0) ));
      fem.transformGeometry (new RigidTransform3d ( new Vector3d (posX, posY, posZ), new AxisAngle(1.0, 0.0, 0.0, 0.0*Math.PI/180.0) ));
      
      return fem;
   }
   
   // --- rigid bodies --- //
   
//   public static RigidBody createRigidBoxTri(double[] xDim, double[] yDim, double[] zDim, double dx, double dy, double dz)
//   {
//      return createRigidBoxTri(
//	    xDim[1]-xDim[0], yDim[1]-yDim[0], zDim[1]-zDim[0], 
//	    deltaToN(xDim[1]-xDim[0],dx), deltaToN(yDim[1]-yDim[0],dy), deltaToN(zDim[1]-zDim[0],dz), 
//	    (xDim[1]+xDim[0])/2.0, (yDim[1]+yDim[0])/2.0, (zDim[1]+zDim[0])/2.0);
//   }
   
   public static RigidBody createRigidBoxTri(double lenX, double lenY, double lenZ, double dx, double dy, double dz, double posX, double posY, double posZ)
   {
      return createRigidBoxTri(lenX,lenY,lenZ, deltaToN(lenX,dx),deltaToN(lenY,dy),deltaToN(lenZ,dz), posX, posY, posZ);
   }
   
   public static RigidBody createRigidBoxTri(double lenX, double lenY, double lenZ, int nX, int nY, int nZ, double posX, double posY, double posZ)
   {
      PolygonalMesh mesh = createFEMBox(lenX, lenY, lenZ, nX, nY, nZ, posX, posY, posZ).getSurfaceMesh();
      mesh.triangulate();
      
      RigidBody rb = new RigidBody();
      rb.setMesh(mesh, null);
      rb.setName("rbBox");
      
      return rb;
   }
   
   public static RigidBody createRigidSurfXTri(double[] x, double[] y, double[] z_x, double lenZ, double dx, double dy, double dz)
   {
      int iX = x.length;
      int iY = y.length;
      double lenX =  x[iX-1] - x[0];
      double lenY =  y[iY-1] - y[0];
      
      return createRigidSurfXTri(x,y,z_x, lenZ, deltaToN(lenX,dx),deltaToN(lenY,dy),deltaToN(lenZ,dz));
   }
   
   public static RigidBody createRigidSurfXTri(double[] x, double[] y, double[] z_x, double lenZ, int nX, int nY, int nZ)
   {
      // creates a surface in which z = z(x)
      
      int iX = x.length;	// iZ better be the same as iX
      int iY = y.length;
      
      double lenX =  x[iX-1] - x[0];
      double lenY =  y[iY-1] - y[0];
      double posX = (x[iX-1] + x[0])/2.0;
      double posY = (y[iY-1] + y[0])/2.0;
      
      RigidBody rb = createRigidBoxTri(lenX, lenY, lenZ, nX, nY, nZ, posX, posY, 0.0);
      
      for (Vertex3d v : rb.getMesh().getVertices())
      {
	 int i=1;
	 double xV = v.getPosition().x;
	 while (x[i] < xV)
	 {
	    if (i == iX-1)
	       break;
	    i++;
	 }
	 double zV = (z_x[i]-z_x[i-1])/(x[i]-x[i-1])*(xV-x[i]) + z_x[i];
	 v.getPosition().z = zV + v.getPosition().z;
      }
      
      return rb;
   }
   
   public static RigidBody createRigidSurfTri(double[] x, double[][] y, double[][] z, double dZ)
   {
      PolygonalMesh mesh = createSurfZ(x,y,z, dZ);
      mesh.triangulate();
      
      RigidBody rb = new RigidBody();
      rb.setMesh(mesh, null);
      rb.setName("rbSurf");
      
      return rb;
   }
   
   public static RigidBody createRigidSurfTri(double[] x, double[] y, double[][] z, double dZ)
   {
      PolygonalMesh mesh = createSurfZ(x,y,z, dZ);
      mesh.triangulate();
      
      RigidBody rb = new RigidBody();
      rb.setMesh(mesh, null);
      rb.setName("rbSurf");
      
      return rb;
   }
   
   public static RigidBody createRigidSurfTri_yConst(double[] x, double[] y, double[] zX, double dZ)
   {
      double[][] z = zX_to_zXY(x,y,zX);
      PolygonalMesh mesh = createSurfZ(x,y,z, dZ);
      mesh.triangulate();
      
      RigidBody rb = new RigidBody();
      rb.setMesh(mesh, null);
      rb.setName("rbSurf");
      
      return rb;
   }
   
   public static PolygonalMesh createSurfZ(double[] x, double[] y, double[][] z, double dZ)
   {
      //maspack.geometry.MeshFactory.createBox()
      PolygonalMesh mesh = new PolygonalMesh();
      int nX = x.length;
      int nY = y.length;
      Vertex3d[][] l1 = new Vertex3d[nX][nY];
      Vertex3d[][] l2 = new Vertex3d[nX][nY];
      
      // add the vertices
      for (int a=0; a<nX; a++)
      {
	 for (int b=0; b<nY; b++)
	 {
	    l1[a][b] = mesh.addVertex(x[a], y[b], z[a][b]-dZ/2.0);
	    l2[a][b] = mesh.addVertex(x[a], y[b], z[a][b]+dZ/2.0);
	 }
      }
      
      // add the faces
      for (int a=1; a<nX; a++)
      {
	 for (int b=1; b<nY; b++)
	 {
	    mesh.addFace(new Vertex3d[]{l1[a][b], l1[a][b-1], l1[a-1][b-1], l1[a-1][b]});
	    mesh.addFace(new Vertex3d[]{l2[a][b], l2[a-1][b], l2[a-1][b-1], l2[a][b-1]});
	    
	    // handle the edges
	    if (a==1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a-1][b-1], l2[a-1][b], l1[a-1][b], l1[a-1][b-1]});
	    }
	    if (b==1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a][b-1], l2[a-1][b-1], l1[a-1][b-1], l1[a][b-1]});
	    }
	    if (a==nX-1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a][b], l2[a][b-1], l1[a][b-1], l1[a][b]});
	    }
	    if (b==nY-1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a][b], l2[a-1][b], l1[a-1][b], l1[a][b]});
	    }
	 }
      }
      return mesh;
   }
   
   public static double[][] zX_to_zXY(double[] x, double[] y, double[] zX)
   {
      int nY = y.length;
      int nX = x.length;
      double[][] z = new double[nX][nY];
      
      for (int a=0; a<nX; a++)
      {
	 for (int b=0; b<nY; b++)
	 {
	    z[a][b] = zX[a];
	 }
      }
      
      return z;
   }
   
   public static PolygonalMesh createSurfZ(double[] x, double[][] y, double[][] z, double dZ)
   {
      //maspack.geometry.MeshFactory.createBox()
      PolygonalMesh mesh = new PolygonalMesh();
      int nX = x.length;
      int nY = y[0].length;
      Vertex3d[][] l1 = new Vertex3d[nX][nY];
      Vertex3d[][] l2 = new Vertex3d[nX][nY];
      
      // add the vertices
      for (int a=0; a<nX; a++)
      {
	 for (int b=0; b<nY; b++)
	 {
	    l1[a][b] = mesh.addVertex(x[a], y[a][b], z[a][b]-dZ/2.0);
	    l2[a][b] = mesh.addVertex(x[a], y[a][b], z[a][b]+dZ/2.0);
	 }
      }
      
      // add the faces
      for (int a=1; a<nX; a++)
      {
	 for (int b=1; b<nY; b++)
	 {
	    mesh.addFace(new Vertex3d[]{l1[a][b], l1[a][b-1], l1[a-1][b-1], l1[a-1][b]});
	    mesh.addFace(new Vertex3d[]{l2[a][b], l2[a-1][b], l2[a-1][b-1], l2[a][b-1]});
	    
	    // handle the edges
	    if (a==1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a-1][b-1], l2[a-1][b], l1[a-1][b], l1[a-1][b-1]});
	    }
	    if (b==1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a][b-1], l2[a-1][b-1], l1[a-1][b-1], l1[a][b-1]});
	    }
	    if (a==nX-1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a][b], l2[a][b-1], l1[a][b-1], l1[a][b]});
	    }
	    if (b==nY-1)
	    {
	       mesh.addFace(new Vertex3d[]{l2[a][b], l2[a-1][b], l1[a-1][b], l1[a][b]});
	    }
	 }
      }
      return mesh;
   }
   
}
