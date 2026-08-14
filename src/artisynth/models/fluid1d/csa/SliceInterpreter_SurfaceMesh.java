package artisynth.models.fluid1d.csa;

import java.util.ArrayList;
import java.util.HashMap;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import artisynth.core.mechmodels.Point;

/*//
 * Author: petera
//*/

public class SliceInterpreter_SurfaceMesh extends SliceInterpreter
{

    PolygonalMesh surface;
    ArrayList<PolygonalMesh> geometries;
    
    ArrayList<MeshMarker> meshMarkers = new ArrayList<>();

    // constructors (same as super class)
    public SliceInterpreter_SurfaceMesh()
    {
        super();
    }

    public SliceInterpreter_SurfaceMesh(String name, Point3d planeCenter, Vector3d planeNormal)
    {
        super(name, planeCenter, planeNormal);
    }

    public SliceInterpreter_SurfaceMesh(String name, ArrayList<Point3d> points)
    {
        super(name, points);
    }

    // implementations
    public void initialize()
    {
        if (findSlice == true)
        {
            geometries = new ArrayList<PolygonalMesh>();
            geometries.add(surface);
            gs.setGeometries(geometries);
        }
        else
        {
            attachPointsAsMarkers();
        }
        update();
    }
    
    public void update()
    {
        
        for (MeshMarker mm : meshMarkers)
        {
            mm.updatePosition();
            //mm.getPosition().inverseTransform( gs.slice.getMeshToWorld() );
        }
        
        super.update();
    }

    public void attachPointsAsMarkers()
    {
        PolylineMesh newMesh = new PolylineMesh();
        
        
        BVFeatureQuery query = new BVFeatureQuery();
        Point3d nearPnt = new Point3d();
        Vector2d uv = new Vector2d();
        
        for (int a=0; a<areaMesh.numLines(); a++)
        {
            Polyline pl = areaMesh.getLines().get(a);
            
            int nPoints = pl.numVertices();
            Vertex3d[] newVerts = new Vertex3d[nPoints+1];
            
            for (int b=0; b<nPoints; b++)
            {
                Vertex3d vert = pl.getVertex(b);
                
                Face face = query.nearestFaceToPoint(nearPnt, uv, surface, vert.getWorldPoint());

                MeshMarker mm = new MeshMarker();
                mm.addWeightedVertex(face.getVertex(0), 1.0-uv.get(0)-uv.get(1));
                mm.addWeightedVertex(face.getVertex(1), uv.get(0) );
                mm.addWeightedVertex(face.getVertex(2), uv.get(1) );
                mm.updatePosition();
                meshMarkers.add(mm);
                
                newVerts[b] = new Vertex3d(mm.getPosition());
                newMesh.addVertex(newVerts[b]);
            }
            newVerts[nPoints] = newVerts[0];
            newMesh.addLine(newVerts);
        }
        areaMesh = newMesh;
    }


    public void setSurface(PolygonalMesh surface)
    {
        this.surface = surface;
    }

}


//A marker for a polygonal mesh
class MeshMarker extends Point
{
    HashMap<Vertex3d, Double> weightedVertices = new HashMap<>();

    public void addWeightedVertex(Vertex3d vert, double weight)
    {
        weightedVertices.put(vert, weight);
    }

    public void clearVertices()
    {
        weightedVertices.clear();
    }

    public void updatePosition()
    {
        Point3d pos = getPosition();
        pos.set(0.0,  0.0,  0.0);
        for (Vertex3d v : weightedVertices.keySet())
        {
            double weight = weightedVertices.get(v);
            pos.x = pos.x + v.getWorldPoint().x * weight;
            pos.y = pos.y + v.getWorldPoint().y * weight;
            pos.z = pos.z + v.getWorldPoint().z * weight;
        }
    }
}
