package artisynth.models.fluid1d;

import java.util.ArrayList;

import artisynth.models.fluid1d.fileIO.CSV;

import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class PolylineUtils 
{
    // rather than extending PolylineMesh, for now this will be a set of centerline utils that are useful
    
    
    // get line section
    
    public static void addPolylineFromCSV(PolylineMesh mesh, String filename)
    {
        CSV csv = new CSV();
        csv.Read(filename);
        
        Vertex3d[] verts = new Vertex3d[csv.nRows]; 
        for (int row=0; row<csv.nRows; row++)
        {
            double[] pnt = csv.data[row];
            verts[row] = mesh.addVertex(pnt[0], pnt[1], pnt[2]);
        }
        mesh.addLine(verts);
    }
    
    public static PolylineMesh resample(PolylineMesh mesh, double dx)
    {
        PolylineMesh rmesh = new PolylineMesh();
        rmesh.setName(mesh.getName());
        for (Polyline pl : mesh.getLines())
        {
            ArrayList<Point3d> oldPoints = new ArrayList<>();
            
            for (Vertex3d v : pl.getVertices())
                oldPoints.add(v.getPosition());
            ArrayList<Point3d> newPoints = resampleOrderedPoints_linear(oldPoints, dx);
            Vertex3d[] newVerts = new Vertex3d[newPoints.size()];
            for (int i=0; i<newPoints.size(); i++)
                newVerts[i] = rmesh.addVertex(newPoints.get(i));
            
            rmesh.addLine(newVerts);
        }
        return rmesh;
    }
    
    public static ArrayList<Point3d> resampleOrderedPoints_linear(ArrayList<Point3d> inputPoints, double dx)
    {
        int nP0 = inputPoints.size(); // number of original points
        
        // define the distance function
        double[] x = new double[nP0];
        x[0] = 0.0;
        for (int a=1; a<nP0; a++)
            x[a] = x[a-1] + inputPoints.get(a).distance(inputPoints.get(a-1));
        
        
        ArrayList<Point3d> outputPoints = new ArrayList<>();
        outputPoints.add(inputPoints.get(0));
        double x_a = x[0]+dx;
        
        while (x_a < x[nP0-1])
        {
            int b=1;
            while ( (x_a > x[b]) && (b<nP0) )
                b++;
            
            Point3d pnt = new Point3d();
            double c = (x_a-x[b-1])/(x[b]-x[b-1]);
            pnt.x = inputPoints.get(b).x * c + inputPoints.get(b-1).x * (1.0 - c);
            pnt.y = inputPoints.get(b).y * c + inputPoints.get(b-1).y * (1.0 - c);
            pnt.z = inputPoints.get(b).z * c + inputPoints.get(b-1).z * (1.0 - c);
            outputPoints.add(pnt);
            
            x_a = x_a + dx;
        }
        outputPoints.add(inputPoints.get(nP0-1));
        int nP1 = outputPoints.size();
        
        if (outputPoints.get(nP1-1).distance(outputPoints.get(nP1-2)) < dx/10.0)
            outputPoints.remove(nP1-2);
        
        return outputPoints;
    }

}
