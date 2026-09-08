package artisynth.models.frank2.registration;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import maspack.geometry.MeshBase;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemFactory.FemElementType;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.CopyableComponent;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank2.frankUtilities.Registration_KnownPoints;

public class RegistrationDemo extends RootModel 
{
    
    public void build (String[] args)
    {
        MechModel mechModel = new MechModel();
        addModel(mechModel);
        
        FemModel3d fem_orig = new FemModel3d();
        FemModel3d fem_wonky = new FemModel3d();   // a transformed version of fem_orig
        FemModel3d fem_restore = new FemModel3d(); // a restoration of fem_orig based on a small selection of registration points
        
        for (FemModel3d fem : new FemModel3d[]{fem_orig, fem_wonky, fem_restore})
            FemFactory.createTorus(fem,    FemElementType.Hex, 1.5, 0.25, 0.5, 20, 10, 3);
        //FemFactory.createTube(fem_orig, FemElementType.Hex, 2.0, 0.25, 0.50, 30, 30, 30);
        
        AffineTransform3d trans = new AffineTransform3d();
        trans.setTranslation( new Vector3d(2.0, 1.0, 1.0) );
        trans.setRotation( new AxisAngle(1.0, 0.0, 0.0, 45.0*Math.PI/180.0) );
        trans.applyScaling(0.05, 0.03, 0.20);
        fem_wonky.transformGeometry(trans);
        fem_restore.transformGeometry(trans);
        
        fem_orig.getRenderProps().setLineColor(Color.green);
        fem_wonky.getRenderProps().setLineColor(Color.red);
        fem_restore.getRenderProps().setLineColor(Color.blue);
        mechModel.addModel(fem_orig);
        mechModel.addModel(fem_wonky);
        mechModel.addModel(fem_restore);
        
        //int[] indices = {0, 50, 100, 200, 300, 400, 500}; // poorly chosen points
        int[] indices = {460, 466, 405, 596, 540, 529, 305, 295}; // 
        ArrayList<Point3d> p_source = new ArrayList<Point3d>();
        ArrayList<Point3d> p_target = new ArrayList<Point3d>();
        for (int j=0; j<indices.length; j++)
        {
            int i=indices[j];
            p_source.add( fem_wonky.getNode(i).getPosition() );
            p_target.add( fem_orig.getNode(i).getPosition() );

            // render selected nodes
            RenderProps rp = new RenderProps();
            rp.setPointColor(Color.orange);
            rp.setPointStyle(PointStyle.SPHERE);
            rp.setPointRadius(0.02);
            fem_orig.getNode(i).setRenderProps(rp);
            fem_wonky.getNode(i).setRenderProps(rp);
        }
        Registration_KnownPoints.transformRBF(p_source, p_target, fem_restore);

        double error = 0.0;
        for (int i=0; i<fem_restore.numNodes(); i++)
            error = error + fem_orig.getNode(i).getPosition().distance(fem_restore.getNode(i).getPosition());
        System.out.printf("total error = %f, average error = %f\n", error, error/fem_restore.numNodes());
        
        // now set up the RBF to restore the original FEM...
//        ArrayList<Point3d> p_all = new ArrayList<Point3d>();
//        for (FemNode3d node : fem_wonky.getNodes())
//            p_all.add(node.getPosition());
//        
//        int[] indices = {0, 50, 100, 200, 300, 400, 500}; // poorly chosen points
//        //int[] indices = {460, 466, 405, 596, 540, 529, 305, 295}; // 
//        ArrayList<Point3d> p_interp = new ArrayList<Point3d>();
//        double[] f_x_known = new double[indices.length];
//        double[] f_y_known = new double[indices.length];
//        double[] f_z_known = new double[indices.length];
//        for (int j=0; j<indices.length; j++)
//        {
//            int i=indices[j];
//            p_interp.add( fem_wonky.getNode(i).getPosition() );
//            f_x_known[j] = fem_orig.getNode(i).getPosition().x - fem_wonky.getNode(i).getPosition().x;
//            f_y_known[j] = fem_orig.getNode(i).getPosition().y - fem_wonky.getNode(i).getPosition().y;
//            f_z_known[j] = fem_orig.getNode(i).getPosition().z - fem_wonky.getNode(i).getPosition().z;
//            
//            // render selected nodes
//            RenderProps rp = new RenderProps();
//            rp.setPointColor(Color.orange);
//            rp.setPointStyle(PointStyle.SPHERE);
//            rp.setPointRadius(0.02);
//            fem_orig.getNode(i).setRenderProps(rp);
//            fem_wonky.getNode(i).setRenderProps(rp);
//        }
//        
//        Kernal kernal = Kernal.multiquadric;
//        RadialBasisFunction rbf_x = new RadialBasisFunction(p_interp, f_x_known, kernal);
//        RadialBasisFunction rbf_y = new RadialBasisFunction(p_interp, f_y_known, kernal);
//        RadialBasisFunction rbf_z = new RadialBasisFunction(p_interp, f_z_known, kernal);
//        double[] f_x = rbf_x.Interpolate(p_all);
//        double[] f_y = rbf_y.Interpolate(p_all);
//        double[] f_z = rbf_z.Interpolate(p_all);
//        
//        
//        double error = 0.0;
//        for (int i=0; i<p_all.size(); i++)
//        {
//            Point3d p = new Point3d(
//                p_all.get(i).x + f_x[i],
//                p_all.get(i).y + f_y[i],
//                p_all.get(i).z + f_z[i]);
//            fem_restore.getNode(i).setPosition(p);
//            
//            error = error + fem_orig.getNode(i).getPosition().distance(fem_restore.getNode(i).getPosition());
//        }
//        System.out.printf("total error = %f, average error = %f\n", error, error/p_all.size());
        
    }
    
    

}
