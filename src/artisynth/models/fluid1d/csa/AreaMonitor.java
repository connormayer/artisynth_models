package artisynth.models.fluid1d.csa;

import java.io.*;
import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.workspace.RootModel;
import artisynth.models.fluid1d.RenderMonitor;

/*//
 * Author: petera
 *  This class is designed to orchestrate the slicing, rendering, and outputting of
 *  cross-sectional area for multiple locations/cutplanes.
//*/

public class AreaMonitor extends MonitorBase  
{
    MechModel mechModel;
    RootModel rootModel;
    ArrayList<FemModel3d> fems;
    ArrayList<RigidBody> rbs;

    ArrayList<SliceInterpreter_Components> calcs = new ArrayList<SliceInterpreter_Components>();

    // options
    boolean initialized = false;
    boolean use3dAreaCalc = false;
    boolean writeToScreen = true;
    boolean writeToFile = false;

    boolean drawArea = true;

    RenderMonitor renderMonitor;
    RenderProps meshRP;
    String outputFile;
    PrintWriter file;

    public AreaMonitor()
    {
    }

    public void initialize()
    {
        for (SliceInterpreter_Components sc : calcs)
        {
            sc.setModels(mechModel, fems, rbs);
            sc.initialize();
        }

        // if the areas are to be rendered, I need to create that here (and update the mesh during apply() )
        if (drawArea == true)
        {
            renderMonitor = new RenderMonitor();
            rootModel.addMonitor(renderMonitor);

            meshRP = new RenderProps();
            meshRP.setVisible(true);
            meshRP.setAlpha(0.05);
            meshRP.setFaceStyle(maspack.render.Renderer.FaceStyle.FRONT_AND_BACK);
            meshRP.setFaceColor(java.awt.Color.cyan);
            meshRP.setDrawEdges(true);
            meshRP.setLineColor(java.awt.Color.cyan);
            meshRP.setLineWidth(2);
            meshRP.setPointColor(java.awt.Color.cyan);
            meshRP.setPointSize(4);

            for (SliceInterpreter_Components sc : calcs)
            {
                PolylineMesh mesh = sc.areaMesh;
                mesh.setRenderProps(meshRP);
                renderMonitor.renderPolylines(mesh);
            }
        }

        if (writeToFile == true)
        {
            try
            {
                file = new PrintWriter(new BufferedWriter(new FileWriter(outputFile, false)));
                file.print("time");
                for (SliceInterpreter_Components sc : calcs)
                {
                    file.print("," + sc.name);
                }
                file.println();
            }
            catch (Exception e)
            {
            }
        }
        initialized = true;

    }

    public void apply(double t0, double t1) 
    {
        if (initialized == false)
        {
            initialize();
        }

        for ( SliceInterpreter_Components sc : calcs )
        {
            sc.update();
        }


        if (drawArea == true)
        {
            renderMonitor.clearPolygonalMeshes();
            renderMonitor.clearPolylineMeshes();
            for (SliceInterpreter_Components sc : calcs)
            {
                PolylineMesh mesh = sc.areaMesh;
                mesh.setRenderProps(meshRP);
                renderMonitor.renderPolylines(mesh);   
            }
        }


        if (writeToScreen == true)
        {
            System.out.print(String.format("time=%f", t1));
            for ( SliceInterpreter_Components sc : calcs )
            {
                System.out.print(String.format(",%s_area=%f", sc.name, sc.area ));
            }
            System.out.println();
        }

        if (writeToFile == true)
        {
            file.printf("%f", t1);
            for (SliceInterpreter_Components sc : calcs)
            {
                file.printf(",%09.9f", sc.area);
                //file.printf(",%f", sc.area);
            }
            file.println();
            file.flush();

        }
    }

    public void setCutplaneByPoints(ArrayList<Point3d> points, String name)
    {
        calcs.add(new SliceInterpreter_Components(name, points));
    }

    public void setCutplaneByPointAndNormal(Point3d planeCenter, Vector3d planeNormal, String name)
    {
        calcs.add(new SliceInterpreter_Components(name, planeCenter, planeNormal));
    }

    public void setModels(RootModel rm, MechModel mm, ArrayList<FemModel3d> fems, ArrayList<RigidBody> rbs)
    {
        rootModel = rm;
        mechModel = mm;
        this.fems = fems;
        this.rbs = rbs;
    }

    public void setOutputWriting(String filename)
    {
        writeToFile = true;
        outputFile = filename;
    }
    
    public void setScreenWriting(boolean writeToScreen)
    {
        this.writeToScreen = writeToScreen;
    }
    
    public void setDrawSlices(boolean drawSlices)
    {
        this.drawArea = drawSlices;
    }
    
    
}
