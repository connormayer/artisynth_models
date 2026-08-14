package artisynth.models.frank2.frankUtilities;

import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.models.frank2.frankUtilities.RadialBasisFunction.Kernal;

/**
 * @author peter
 *
 */
public class Registration_KnownPoints 
{
    // TODO: pass an RBF, such that all RBF options can be used?
    
    static Kernal kernal_default = Kernal.linear;
    Kernal kernal = kernal_default;
    RadialBasisFunction rbf_x;
    RadialBasisFunction rbf_y;
    RadialBasisFunction rbf_z;
    
    public Registration_KnownPoints(ArrayList<Point3d> pSource, ArrayList<Point3d> pTarget, Kernal kernal)
    {
        initialize(pSource, pTarget, kernal);
    }
    
    void initialize(ArrayList<Point3d> pSource, ArrayList<Point3d> pTarget, Kernal kernal)
    {
        if (pSource.size() != pTarget.size())
        {
            System.err.println("RBF interpolation: source and target points must have the same number of points.");
        }
        int nPoints = pSource.size();
        
        double[] f_x_known = new double[nPoints];
        double[] f_y_known = new double[nPoints];
        double[] f_z_known = new double[nPoints];
        for (int j=0; j<nPoints; j++)
        {
            f_x_known[j] = pTarget.get(j).x - pSource.get(j).x;
            f_y_known[j] = pTarget.get(j).y - pSource.get(j).y;
            f_z_known[j] = pTarget.get(j).z - pSource.get(j).z;
        }
        
        rbf_x = new RadialBasisFunction(pSource, f_x_known, kernal);
        rbf_y = new RadialBasisFunction(pSource, f_y_known, kernal);
        rbf_z = new RadialBasisFunction(pSource, f_z_known, kernal);
    }
    
    public ArrayList<Point3d> interpolate(ArrayList<Point3d> pointSet)
    {
        double[] f_x = rbf_x.Interpolate(pointSet);
        double[] f_y = rbf_y.Interpolate(pointSet);
        double[] f_z = rbf_z.Interpolate(pointSet);
        
        ArrayList<Point3d> p_trans = new ArrayList<Point3d>(pointSet.size());
        for (int i=0; i<pointSet.size(); i++)
        {
            Point3d p = new Point3d(
                pointSet.get(i).x + f_x[i],
                pointSet.get(i).y + f_y[i],
                pointSet.get(i).z + f_z[i]);
            p_trans.add(p);
        }
        return p_trans;
    }
    
    /**
     * Transforms the fem mesh using the transform defined by pSource to pTarget. 
     */
    public void interpolate(FemModel3d fem)
    {
        // transform the FemNodes
        ArrayList<Point3d> p_all = new ArrayList<Point3d>(fem.numNodes());
        for (FemNode3d node : fem.getNodes())
            p_all.add(node.getPosition());
        
        ArrayList<Point3d> p_trans = interpolate(p_all);
        for (int i=0; i<p_all.size(); i++)
        {
            fem.getNode(i).setPosition(p_trans.get(i));
        }
        
        // transform the FemMarkers
        ArrayList<Point3d> p_markers = new ArrayList<Point3d>(fem.markers().size());
        for (FemMarker m : fem.markers())
            p_markers.add(m.getPosition());
        
        ArrayList<Point3d> p_markers_x = interpolate(p_markers);
        for (int i=0; i<p_markers.size(); i++)
        {
            fem.markers().get(i).setPosition(p_markers_x.get(i));
        }
    }
    
    /**
     * Transforms the fem mesh using the transform defined by pSource to pTarget. 
     */
    public void interpolate(PolygonalMesh mesh)
    {   
        ArrayList<Point3d> p_all = new ArrayList<Point3d>(mesh.numVertices());
        for (Vertex3d vert : mesh.getVertices())
            p_all.add(vert.getPosition());
        
        ArrayList<Point3d> p_trans = interpolate(p_all);
        for (int i=0; i<p_all.size(); i++)
        {
            mesh.getVertex(i).setPosition(p_trans.get(i));
        }
    }
    
    /**
     * Defines a transform from pSource to pTarget using RBFs, and then transforms pointSet accordingly. 
     * @param pSource are known points (in same space as pointSet)
     * @param pTarget are corresponding points in the target space
     * @param pointSet is the complete point set to be transformed
     */
    public static ArrayList<Point3d> transformRBF(ArrayList<Point3d> pSource, ArrayList<Point3d> pTarget, ArrayList<Point3d> pointSet)
    {
        Registration_KnownPoints rbfReg = new Registration_KnownPoints(pSource, pTarget, kernal_default);
        return rbfReg.interpolate(pointSet);
        /*
        if (pSource.size() != pTarget.size())
        {
            System.err.println("RBF interpolation: source and target points must have the same number of points.");
        }
        int nPoints = pSource.size();
        
        double[] f_x_known = new double[nPoints];
        double[] f_y_known = new double[nPoints];
        double[] f_z_known = new double[nPoints];
        for (int j=0; j<nPoints; j++)
        {
            f_x_known[j] = pTarget.get(j).x - pSource.get(j).x;
            f_y_known[j] = pTarget.get(j).y - pSource.get(j).y;
            f_z_known[j] = pTarget.get(j).z - pSource.get(j).z;
        }
        
        RadialBasisFunction rbf_x = new RadialBasisFunction(pSource, f_x_known, kernal);
        RadialBasisFunction rbf_y = new RadialBasisFunction(pSource, f_y_known, kernal);
        RadialBasisFunction rbf_z = new RadialBasisFunction(pSource, f_z_known, kernal);

        double[] f_x = rbf_x.Interpolate(pointSet);
        double[] f_y = rbf_y.Interpolate(pointSet);
        double[] f_z = rbf_z.Interpolate(pointSet);
        
        ArrayList<Point3d> p_trans = new ArrayList<Point3d>(pointSet.size());
        for (int i=0; i<pointSet.size(); i++)
        {
            Point3d p = new Point3d(
                pointSet.get(i).x + f_x[i],
                pointSet.get(i).y + f_y[i],
                pointSet.get(i).z + f_z[i]);
            p_trans.add(p);
        }
        return p_trans;
        */
    }
    
    /**
     * Defines a transform from pSource to pTarget using RBFs, and then transforms fem accordingly. 
     * @param pSource are known points (in same space as fem)
     * @param pTarget are corresponding points in the target space
     * @param fem mesh will be transformed to the defined transform
     */
    public static void transformRBF(ArrayList<Point3d> pSource, ArrayList<Point3d> pTarget, FemModel3d fem)
    {
        Registration_KnownPoints rbfReg = new Registration_KnownPoints(pSource, pTarget, kernal_default);
        rbfReg.interpolate(fem);
   
        /*//
        ArrayList<Point3d> p_all = new ArrayList<Point3d>(fem.numNodes());
        for (FemNode3d node : fem.getNodes())
            p_all.add(node.getPosition());
        
        ArrayList<Point3d> p_trans = transformRBF(pSource, pTarget, p_all);
        for (int i=0; i<p_all.size(); i++)
        {
            fem.getNode(i).setPosition(p_trans.get(i));
        }
        //*/
    }
    
    /**
     * Defines a transform from pSource to pTarget using RBFs, and then transforms the polygonal mesh accordingly. 
     * @param pSource are known points (in same space as the mesh)
     * @param pTarget are corresponding points in the target space
     * @param mesh will be transformed to the defined transform
     */
    public static void transformRBF(ArrayList<Point3d> pSource, ArrayList<Point3d> pTarget, PolygonalMesh mesh)
    {
        Registration_KnownPoints rbfReg = new Registration_KnownPoints(pSource, pTarget, kernal_default);
        rbfReg.interpolate(mesh);
        
        /*
        ArrayList<Point3d> p_all = new ArrayList<Point3d>(mesh.numVertices());
        for (Vertex3d vert : mesh.getVertices())
            p_all.add(vert.getPosition());
        
        ArrayList<Point3d> p_trans = transformRBF(pSource, pTarget, p_all);
        for (int i=0; i<p_all.size(); i++)
        {
            mesh.getVertex(i).setPosition(p_trans.get(i));
        }
        //*/  
    }

}
