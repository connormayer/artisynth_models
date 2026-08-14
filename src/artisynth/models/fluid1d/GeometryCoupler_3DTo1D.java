package artisynth.models.fluid1d;

import java.util.ArrayList;

import artisynth.core.mechmodels.Marker;
import artisynth.models.fluid1d.csa.AreaCalculator;
import artisynth.models.fluid1d.csa.SliceGeometry;
import artisynth.models.fluid1d.csa.SliceInterpreter;
import artisynth.models.fluid1d.csa.SliceInterpreter_Components;
import artisynth.models.fluid1d.csa.SliceInterpreter.OrderingMethod;
import maspack.matrix.*;
import maspack.geometry.*;

public class GeometryCoupler_3DTo1D implements GeometryCoupler 
{
    /*//
     * input: fluid and structure solutions
     * output: calculates the area and perimeter of the structure (which is stored in the fluidSolution geometry)
     * 
     * initialization:
     *    calculates the intersections of the cutplanes with the geometry
     *    reduces the full slice to the 'useful' slice needed to calculate the area
     *    builds a list of points (interpolationPoints) used in the area calculations
     *    
     * update: 
     *    Does not re-calculate the slice. It just updates the area and perimeter calculations. 
     *    If re-slicing is needed, re-intialize
   //*/ 

    StructureSolution inputSolution;
    FluidSolution_1D outputSolution;

    // the geometries
    PolygonalMesh inputGeometry;			// perhaps this should be Geometry_3D
    Geometry_1D outputGeometry;

    Centerline centerline;
    ArrayList<SliceGeometry> slicers;
    ArrayList<PolylineMesh> slices;

    boolean sliceEachStep = false;
    boolean sliceThisStep = true;

    public enum OrderingMethod {ClosedContours, LineOfSight_3dRays, LineOfSight_2dExact};
    OrderingMethod orderingMethod = OrderingMethod.LineOfSight_2dExact;  // TODO: shouldn't be default anymore?

    public void initialize()
    {
        update();
        System.out.println("Geometry Coupler Initialized");
    }

    public void update()
    {
        int nVertices = centerline.getVertices().size();

        if (sliceThisStep == true)
        {
            slicers = new ArrayList<SliceGeometry>(nVertices);
            slices = new ArrayList<PolylineMesh>(nVertices);

            for (int a=0; a<nVertices; a++)
            {
                PolylineMesh areaMesh;      // the reduced slice (ordered points)

                // Moving the next 2 lines out of the loop would make vtk faster, but must include something like "setSlice" to create a new slice
                SliceGeometry slicer = new SliceGeometry();
                slicer.setPlane(centerline.getVertices().get(a), centerline.findDirectionAtVertex(a));
                slicer.setGeometry(inputGeometry);
                slicer.update();

                Point3d pointXY = new Point3d( slicer.getPlaneOrigin() );      
                pointXY.inverseTransform( slicer.getSlice().getMeshToWorld() );

                if (orderingMethod == OrderingMethod.ClosedContours) 
                    areaMesh = AreaCalculator.findSlice_Contours(slicer.getSlice(), pointXY); // find the main contour (the innermost contour which contains the centerline point)
                else if (orderingMethod == OrderingMethod.LineOfSight_2dExact)
                    areaMesh = AreaCalculator.findVisibleSlice_2dExact(slicer.getSlice(), pointXY);
                //else if (orderingMethod == OrderingMethod.LineOfSight_3dRays)  // Not working! But this wouldn't go here as it doesn't us GeomSlice at all.
                else
                    areaMesh = new PolylineMesh();

                // if the slice is not recalculated each time step, then attach marker points which will be used to track slice motion
                if (sliceEachStep == false)
                    areaMesh = attachPointsAsMarkers(areaMesh);

                double[] ap = updateArea(areaMesh);  // update perimeter, area
                outputGeometry.setArea(a, ap[0]);
                outputGeometry.setPerimeter(a, ap[1]);

                slicers.add(slicer);
                slices.add(areaMesh);
            }

            if (sliceEachStep == false)
                sliceThisStep = false;
        }
        else
        {
            for (int a=0; a<nVertices; a++)
            {
                double[] ap = updateArea(slices.get(a));  // update perimeter, area
                outputGeometry.setArea(a, ap[0]);
                outputGeometry.setPerimeter(a, ap[1]);
            }
        }

    }

    public static double[] updateArea(PolylineMesh areaMesh)
    {
        // TODO: move to area calculator?
        double area = 0.0;
        double perimeter = 0.0;

        for (int a=0; a<areaMesh.numLines(); a++)
        {
            ArrayList<Point3d> points = new ArrayList<Point3d>();
            Polyline pl = areaMesh.getLines().get(a);

            for (int b=0; b<pl.numVertices (); b++)
                points.add(pl.getVertex(b).getWorldPoint());

            double areaP  = AreaCalculator.calcArea3d(points);
            double perimP = AreaCalculator.calcPerimeter3d (points);

            if (a==0)
            {
                area = areaP;
                perimeter = perimP;
            }
            else
            {
                area = area - areaP;                // an internal area reduces the cross-sectional area...
                perimeter = perimeter + perimP;     // ...yet increases the wetted perimeter.
            }
        }
        return new double[]{area, perimeter};
    }

    public PolylineMesh attachPointsAsMarkers(PolylineMesh areaMesh)
    {
        //ArrayList<Point3d> points = new ArrayList<Point3d>(areaMesh.numVertices()); 
        PolylineMesh newMesh = new PolylineMesh();

        for (Polyline pl : areaMesh.getLines())
        {
            int nPoints = pl.numVertices();
            Vertex3d[] verts = new Vertex3d[nPoints];

            for (int a=0; a<nPoints; a++)
            {
                Point3d p = pl.getVertex(a).getWorldPoint();
                Marker m = inputSolution.createMarkerPoint( p );            // if face is known too, use createMarkerPoint(point,face)
                Vertex3d v = newMesh.addVertex(m.getPosition(), true);      // true to keep the reference
                verts[a] = v;
            }
            newMesh.addLine(verts);

        }
        // TODO: could it be a problem that the new areaMesh no longer uses local cutplane coordinates?
        //newMesh.setMeshToWorld(areaMesh.getMeshToWorld());
        return newMesh;
    }

    //   public void setSlices(ArrayList<PolylineMesh> slices)
    //   {
    //      // rather than calculating the slice, they could be manually set...
    //   }


    public void setCenterline(Centerline centerline)
    {
        this.centerline = centerline;
    }

    public Geometry_1D getOutputGeometry()
    {
        return outputGeometry;
    }

    public void setInputSolution(StructureSolution ss)
    {
        this.inputSolution = ss;
        inputGeometry = ss.getGeometry();
    }

    public void setOutputSolution(FluidSolution_1D fs)
    {
        this.outputSolution = fs;
        outputGeometry = fs.getGeometry();
        setCenterline(outputGeometry.getCenterline());
    }

    public ArrayList<PolylineMesh> getSlices()
    {
        return slices;
    }

}
