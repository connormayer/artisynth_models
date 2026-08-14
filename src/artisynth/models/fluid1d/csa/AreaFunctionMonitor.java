package artisynth.models.fluid1d.csa;

import java.awt.Color;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.matrix.Point3d;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import artisynth.core.modelbase.MonitorBase;
import artisynth.models.fluid1d.fileIO.CSV;
import artisynth.models.fluid1d.plot2d.Plot2d;
import artisynth.models.fluid1d.plot2d.Plot2dAxes;
import artisynth.models.fluid1d.plot2d.Plot2dSeries;


public class AreaFunctionMonitor extends MonitorBase
{
    protected PolygonalMesh surface;
    protected Polyline centerline;
    protected int nPoints;
    protected ArrayList<SliceInterpreter> slices = new ArrayList<>();
    
    protected double[] areas;
    protected double[] dists;
    
    protected boolean sliceEachStep = false;
    protected boolean use3dArea = true;
    boolean updateCenterlineBySliceCentroids = false;
    SliceInterpreter.OrderingMethod om = SliceInterpreter.OrderingMethod.ClosedContours;
    boolean discardInvalidSlices = false; //TODO: implement a feature that recognizes an invalid value and cleans/removes it
    
    // plotting options
    protected boolean plotAreaFunction = true;
    protected Plot2d plot = new Plot2d();
    
    boolean printToScreen = false;
    boolean writeToFile = false;
    
    // saving options
    // save to csv --> t,a0,a1...aN
    String outputDir;
    String outputBasename;

    int nStep = 0;

    RenderProps meshRP = new RenderProps();
    RenderProps meshRP_raw = new RenderProps();

    public AreaFunctionMonitor()
    {
        // --- define default rendering --- //
        meshRP.setVisible(true);
        meshRP.setDrawEdges(true);
        meshRP.setLineColor(Color.orange);
        meshRP.setLineWidth(5);

        meshRP_raw = meshRP.clone();
        meshRP_raw.setLineColor(Color.red);
        meshRP_raw.setVisible(false);
    }
    
    //public void initialize() throws SocketException, UnknownHostException
    public void initialize()
    {  

        areas = new double[nPoints];
        dists = new double[nPoints];
        double len = centerline.computeLength();

        for (int a=0; a<nPoints; a++)
        {
            if (a>0)
                dists[a] = dists[a-1] + centerline.getVertex(a).getWorldPoint().distance( centerline.getVertex(a-1).getWorldPoint() );
            
            SliceInterpreter_SurfaceMesh si = new SliceInterpreter_SurfaceMesh();
            si.setCutplaneByPointAndNormal(
                centerline.getVertex(a).getWorldPoint(), 
                centerline.interpolateTangent(dists[a]/len) );
            si.setSurface(surface);
            si.setSliceEachStep(sliceEachStep);
            si.setUse3dAreaCalcs(use3dArea);
            si.initialize();
            
            slices.add(si);
            areas[a] = si.area;
        }
        
        // define a plot to show the CSA
        if (plotAreaFunction == true)
        {
            Plot2dSeries series1 = new Plot2dSeries("area function", dists, areas);
            Plot2dAxes axis = new Plot2dAxes();
            axis.addSeries(series1);
            axis.updateRangeFromData();
            plot.addAxis(axis);
            plot.buildPlot(700, 0, 500, 200); // build plot with window & location
            plot.update();
        }

    }
    
    @Override
    public void apply(double t0, double t1) 
    {
        areas = new double[nPoints];
        dists = new double[nPoints];

        for (int a=0; a<nPoints; a++)
        {
            if (a>0)
                dists[a] = dists[a-1] + centerline.getVertex(a).getWorldPoint().distance( centerline.getVertex(a-1).getWorldPoint() );
            
            SliceInterpreter si = slices.get(a);
            if (sliceEachStep == true)
                si.setCutplaneByPointAndNormal(centerline.getVertex(a).getWorldPoint(), centerline.interpolateTangent(dists[a]/centerline.computeLength()));
            si.update();

            areas[a] = si.area;

            if (updateCenterlineBySliceCentroids == true)
            {
                // define the centerline vertex at the centroid
                Point3d centroid = new Point3d();
                si.getSlice().computeCentroid(centroid);
                centerline.getVertex(a).getPosition().set(centroid);
            }
        }

        if (plotAreaFunction == true)
        {
            plot.getAxis(0).getSeries().get(0).x = dists;
            plot.getAxis(0).getSeries().get(0).y = areas;
            plot.getAxis(0).updateRangeFromData();
            plot.update();
        }

        if (printToScreen == true)
            printAreas();

        if (writeToFile == true)
            saveAreaFunction( String.format("%s%s_%05.3f.csv", outputDir, outputBasename, t0) );


        nStep++;

    }

    public void printAreas()
    {
        System.out.printf("area (%d, %05.3f): ", areas.length, dists[dists.length-1]);
        for (double area : areas)
            System.out.printf("%07.5f, ",  area);
        System.out.println();
    }
    
    public void saveAreaFunction(String filename)
    {
        double[][] data = new double[areas.length][];
        for (int a=0; a<areas.length; a++)
            data[a] = new double[]{dists[a], areas[a]};
        CSV.Write(filename, new String[]{"x", "area"}, data);
    }

    @Override
    public void prerender (RenderList list) 
    {
        for (SliceInterpreter sc : slices)
        {
            PolylineMesh mesh = sc.areaMesh;
            mesh.setRenderProps(meshRP);
            list.addIfVisible(mesh);

            PolylineMesh mesh_raw = sc.getRawSlice();
            mesh_raw.setRenderProps(meshRP_raw);
            list.addIfVisible(mesh_raw);
        }

        super.prerender(list);
    }

    public double[] getAreas()
    {
        return areas;
    }
    
    public double[] getDists()
    {
        return dists;
    }
    
    public int getNumPoints()
    {
        return nPoints;
    }
    
    public void setCenterline(Polyline centerline)
    {
        this.centerline = centerline;
        nPoints = centerline.numVertices();
    }
    
    public Polyline getCenterline()
    {
        return centerline;
    }
    
    public void setGeometry(PolygonalMesh surface)
    {
        this.surface = surface;
    }
    
    public PolygonalMesh getGeometry()
    {
        return surface;
    }
    
    public void setSlicingMethod(SliceInterpreter.OrderingMethod slicingMethod)
    {
        this.om = slicingMethod;
    }
    
    public void setSliceEachStep(boolean sliceEachStep)
    {
        this.sliceEachStep = sliceEachStep;
    }
    
    public void set3dAreaCalculation(boolean use3dArea)
    {
        this.use3dArea = use3dArea;
    }
    
    public void setUpdateCenterlineByCentroids(boolean updateCenterline)
    {
        this.updateCenterlineBySliceCentroids = updateCenterline;
    }
    
    public Plot2d getPlot()
    {
        return plot;
    }
    
    public void setPrintToScreen(boolean printToScreen)
    {
        this.printToScreen = printToScreen;
    }
    
    public void setOutputPath(String outputDir, String outputBasename)
    {
        writeToFile = true;
        this.outputDir = outputDir;
        this.outputBasename = outputBasename;
    }
    
    public RenderProps getRawSliceRenderProps()
    {
        return meshRP_raw;
    }
    
    public RenderProps getFinalSliceRenderProps()
    {
        return meshRP;
    }
    
    public void setRawSliceRenderProps(RenderProps renderProps)
    {
        this.meshRP_raw = renderProps;
    }
    
    public void setFinalSliceRenderProps(RenderProps renderProps)
    {
        this.meshRP = renderProps;
    }
}
