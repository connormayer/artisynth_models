package artisynth.models.fluid1d;

import java.awt.Color;
import java.util.ArrayList;

import maspack.matrix.*;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;
import maspack.geometry.*;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.MonitorBase;


/*//
 * Author: petera
 * This class is designed to offer quick and easy rendering of points, lines, etc...
//*/

// The required functionalities:
// part of mech model, renderable
//	* simple, just a one-time modification to the render props
//	* static methods
// part of mech model, non-rendering
//	* easy, make into renderable and add in prerender
// not part of mech model (hence non-rendering)
//	* complicated, the objects definition needs to be updated each step (or before each rendering)
//	* just render once, don't keep in que
//	* example: centroid of dynamic face.
//	* dealt with in prerender

public class RenderMonitor extends MonitorBase 
{
    class PointSet
    {
        ArrayList<Point3d> points;
        RenderProps rp;

        public PointSet()
        {
            points = new ArrayList<Point3d>();
            rp = new RenderProps();
        }

        public PointSet(ArrayList<Point3d> points, RenderProps renderProps)
        {
            this.points = points;
            rp = renderProps;
        }

        public PointSet(ArrayList<Point3d> points, double size, Color color)
        {
            this.points = points;

            rp = new RenderProps();
            rp.setPointColor(color);
            rp.setPointStyle(PointStyle.SPHERE);
            rp.setPointRadius(size);
            rp.setVisible(true);
        }

        public ArrayList<Point> getRenderablePoints()
        {
            ArrayList<Point> rps = new ArrayList<Point>(points.size());
            for (Point3d p : points)
            {
                Point point = new Point(p);
                point.setRenderProps(rp);
                rps.add(point);
            }
            return rps;
        }


    }

    ArrayList<PointSet> pointArr = new ArrayList<PointSet>();
    ArrayList<PolygonalMesh> meshes = new ArrayList<PolygonalMesh>();
    ArrayList<PolylineMesh> polylines = new ArrayList<PolylineMesh>();

    GeometryCoupler_3DTo1D geometryCoupler = null;
    RenderProps geometryCouplerRP = null;

    //ArrayList<? extends MeshBase> meshes = new ArrayList<MeshBase>();  // would be nice but can't add to it...

    public void apply(double t0, double t1) 
    {
        // do I need to use prerender, or can I achieve() all of that in apply()?
        // update all face centroids
        //render(artisynth.core.driver.Main.getMain().getRootModel().getMainViewer()); // don't do this...
    }

    public void prerender(RenderList list)
    {
        super.prerender(list);

        for (PointSet ps : pointArr)
            list.addIfVisibleAll(ps.getRenderablePoints());
        for (MeshBase mesh : meshes)
            list.addIfVisible(mesh);
        for (MeshBase mesh : polylines)
            list.addIfVisible(mesh);

        if (geometryCoupler != null)
        {
            for (MeshBase mesh : geometryCoupler.slices)
            {
                mesh.setRenderProps(geometryCouplerRP);
                list.addIfVisible(mesh);
            }
        }
        //      for (MeshBase mesh : centerlineSlices)
        //         list.addIfVisible(mesh); 

    }

    // this doesn't work
    //   public void render (Renderer r) 
    //   {
    //      
    //      for (PointSet ps : pointArr)
    //      {
    //	 for (Point3d p : ps.points)
    //	    r.drawPoint(ps.rp, new float[]{(float) p.x, (float) p.y, (float) p.z}, false);
    //      }
    //   }


    // TODO: add "render once" option the "mark" methods
    public void markLine(ArrayList<Point3d> linePoints, Color color)
    {      
        PolylineMesh mesh = new PolylineMesh();
        int[] indices = new int[linePoints.size()];
        for (int i=0; i<linePoints.size(); i++)
        {
            mesh.addVertex(linePoints.get(i), true);
            indices[i] = i;
        }
        mesh.addLine(indices);

        RenderProps rp = mesh.getRenderProps();
        rp.setPointColor(color);
        rp.setPointSize(5);
        //      rp.setPointStyle(PointStyle.SPHERE);
        //      rp.setPointRadius(scale);
        rp.setLineColor(color);
        rp.setLineWidth(3);

        renderPolylines(mesh);
    }

    public void markPoints(ArrayList<Point3d> points, double size, Color color)
    {      
        PointSet ps = new PointSet(points, size, color);
        pointArr.add(ps);
    }

    public void markFaces(ArrayList<Face> faces, double scale, Color color)
    {
        // TODO: this needs to update the centroid with each time step...
        ArrayList<Point3d> points = new ArrayList<Point3d>(faces.size());
        for (Face f : faces)
        {
            Point3d c = new Point3d();
            f.computeWorldCentroid(c);
            points.add(c);
        }

        markPoints(points, scale, color);
    }

    public void renderMesh(PolygonalMesh mesh, RenderProps rp)
    {
        mesh.setRenderProps(rp);
        meshes.add(mesh);
    }

    public void renderMesh(PolygonalMesh mesh)
    {
        meshes.add(mesh);
    }

    public void renderPolylines(PolylineMesh mesh, RenderProps rp)
    {
        mesh.setRenderProps(rp);
        polylines.add(mesh);
    }

    public void renderPolylines(PolylineMesh mesh)
    {
        polylines.add(mesh);
    }

    public void renderGeometryCouplerSlices(GeometryCoupler_3DTo1D geomCoupler)
    {
        RenderProps rp = new RenderProps();
        rp.setVisible(true);
        renderGeometryCouplerSlices(geomCoupler, rp);
    }

    public void renderGeometryCouplerSlices(GeometryCoupler_3DTo1D geomCoupler, RenderProps rp)
    {
        geometryCoupler = geomCoupler;
        geometryCouplerRP = rp;
    }

    public void clearPolygonalMeshes()
    {
        meshes.clear();
    }

    public void clearPolylineMeshes()
    {
        polylines.clear();
    }

    public void renderLine(ArrayList<? extends Point> linePoints, Color color)
    {
        ArrayList<Point3d> pnts = new ArrayList<Point3d>(linePoints.size());
        for ( Point p : linePoints)
            pnts.add(p.getPosition());

        markLine(pnts, color);
    }

    public static void renderPoints(ArrayList<? extends Point> points, double scale, Color color)
    {
        for (Point point : points)
        {
            if (point.getRenderProps() == null)
                point.setRenderProps(new RenderProps());
            point.getRenderProps().setPointColor(color);
            point.getRenderProps().setPointRadius(scale);
            point.getRenderProps().setPointStyle(PointStyle.SPHERE);
        }
    }

    public static void RenderMarkers(ArrayList<Marker> markers, double scale, Color color)
    {
        renderPoints(markers, scale, color);
    }

    public static void renderNodes(ArrayList<FemNode3d> nodes, double scale, Color color)
    {
        renderPoints(nodes, scale, color);
    }

    public static void renderStaticNodes(FemModel3d model, double scale, Color color)
    {
        ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
        for (FemNode3d node : model.getNodes())
        {
            if (node.isDynamic() == false)
            {
                nodes.add(node);
            }
        }
        renderNodes(nodes, scale, color);
    }

    public static void renderAttachedNodes(FemModel3d model, double scale, Color color)
    {
        ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
        for (FemNode3d node : model.getNodes())
        {
            if (node.isAttached() == true)
            {
                nodes.add(node);
            }
        }
        renderNodes(nodes, scale, color);
    }
    
    public static void drawExternalForces(Renderer renderer, FemModel3d fem, Color color, double scale)
    {
        RenderProps rp = new RenderProps();
        rp.setLineColor(color);
        
        RenderProps rp_tail = new RenderProps();
        rp_tail.setPointColor(Color.red);
        rp_tail.setPointSize(7);
        RenderProps rp_head = new RenderProps();
        rp_head.setPointColor(Color.orange);
        rp_head.setPointSize(3);
        
        for (FemNode3d node : fem.getNodes())
        {
            if (node.getExternalForce().norm() < 1e-10)
                continue;
            Point3d p1 = node.getPosition();
            Point3d p2 = new Point3d(node.getExternalForce());
            p2.scale(scale);
            //p2.add(p1);
            p2.sub(p1, p2);
            
            float[] coords1 = {(float)p1.x, (float)p1.y, (float)p1.z};
            float[] coords2 = {(float)p2.x, (float)p2.y, (float)p2.z};
            renderer.drawLine(rp, coords1, coords2, null, true, false);
            renderer.drawPoint(rp_tail, coords2, false);
            renderer.drawPoint(rp_head, coords1, false);
//            renderer.drawArrow(rp, 
//                new float[]{(float)p1.x, (float)p1.y, (float)p1.z}, 
//                new float[]{(float)p2.x, (float)p2.y, (float)p2.z}, false, false);
        }
    }
    
    public static void drawPoints(Renderer renderer, Iterable<Point3d> points, Color color, int size)
    {
        RenderProps rp = new RenderProps();
        rp.setPointColor(color);
        rp.setPointSize(size);
        
        for (Point3d p : points)
        {
            float[] coords1 = {(float)p.x, (float)p.y, (float)p.z};
            renderer.drawPoint(rp, coords1, false);
        }
    }
    
    public static void drawRay(Renderer renderer, Point3d base, Vector3d vector, double scale, Color color)
    {
        RenderProps rp = new RenderProps();
        rp.setPointColor(color);
        rp.setLineColor(color);
        rp.setPointSize(5);
        rp.setLineWidth(3);
        
        Point3d head = new Point3d();
        head.add(base, vector);
        head.scale(scale);
        
        float[] coords1 = {(float)base.x, (float)base.y, (float)base.z};
        float[] coords2 = {(float)head.x, (float)head.y, (float)head.z};
        renderer.drawPoint(rp, coords1, false);
        renderer.drawLine(rp, coords1, coords2, null, false, false);
    }

}
