package artisynth.models.fluid1d.csa;

import java.util.ArrayList;

import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import artisynth.core.femmodels.PointSkinAttachment;
import artisynth.core.femmodels.SkinMeshBody;

/*//
 * Author: petera
//*/

public class SliceInterpreter_SkinMesh extends SliceInterpreter
{

    SkinMeshBody surface;
    ArrayList<PolygonalMesh> geometries;

    // constructors (same as super class)
    public SliceInterpreter_SkinMesh()
    {
        super();
    }

    public SliceInterpreter_SkinMesh(String name, Point3d planeCenter, Vector3d planeNormal)
    {
        super(name, planeCenter, planeNormal);
    }

    public SliceInterpreter_SkinMesh(String name, ArrayList<Point3d> points)
    {
        super(name, points);
    }

    // implementations
    public void initialize()
    {
        if (findSlice == true)
        {
            geometries = new ArrayList<PolygonalMesh>();
            geometries.add((PolygonalMesh)surface.getMesh());
            gs.setGeometries(geometries);
        }
        else
        {
            attachPointsAsMarkers();
        }
        update();
    }

    //   public void update()
    //   {
    //      // many options from the area calculator could be added here if needed...
    //      if (findSlice == true)
    //      {
    //         gs.update();                           // update the slice of the entire geometry
    //         
    //         areaMesh = new PolylineMesh();
    //         areaMesh.setMeshToWorld( gs.slice.getMeshToWorld() );
    //	 
    //	 Point3d pointXY = new Point3d( gs.getPlaneOrigin() );      
    //	 pointXY.inverseTransform( gs.slice.getMeshToWorld() );
    //	 
    //	 // find the main contour (the innermost contour which contains the centerline point)
    //	 int iMainCont = -1;
    //	 int numContours = gs.slice.numLines();
    //	 for (int n=0; n<numContours; n++)
    //	 {
    //	    Polyline pl = gs.slice.getLines().get(n);
    //            if (isPolylineClosed(pl) == true)
    //            {
    //               if (isPointInsidePolyline(pointXY, pl) == true)
    //               {
    //                  if (areaMesh.numLines() == 0)
    //                  {
    //                     areaMesh.addLine(pl);
    //                     iMainCont=n;
    //                  }
    //                  else if ( isPolylineInsidePolyline(pl, areaMesh.getLines().get(0)) )
    //                  {
    //                     areaMesh.getLines().remove(0);
    //                     areaMesh.addLine(pl);
    //                     iMainCont=n;
    //                  }
    //               }
    //            }
    //	 }
    //	 // TODO: to speed this code up, keep a list of rejected contours (open, outside of main, etc...)
    //	 if (areaMesh.numLines() > 0)
    //	 {
    //	    // find any internal contours (contours located inside the main contour, but not containing the centerline point)
    //	    for (int n=0; n<numContours; n++)
    //	    {
    //	       if (n != iMainCont)
    //	       {
    //	          Polyline pl = gs.slice.getLines().get(n);
    //	          if (isPolylineClosed(pl) == true)
    //	          {
    //	             if (isPolylineInsidePolyline(pl, areaMesh.getLines().get(0)) == true)
    //	             {
    //	                areaMesh.addLine(pl);
    //	             }
    //	          }
    //	       }
    //	    }
    //	 }
    //	 updateArea();
    //
    //	 //*//
    //	 if (sliceEachStep == false)
    //	 {
    //	    findSlice = false;
    //	    attachPointsAsMarkers();
    //	 }
    //	 //*/
    //      }
    //      else
    //      {
    //	 updateArea();
    //      }
    //   }

    public void attachPointsAsMarkers()
    {
        for (int a=0; a<areaMesh.numLines(); a++)
        {
            Polyline pl = areaMesh.getLines().get(a);

            for (int b=0; b<pl.numVertices (); b++)
            {  
                //PointSkinAttachment psa = new PointSkinAttachment();
                //psa.setBasePosition (pl.getVertex(b).pnt);
                surface.addMarker (pl.getVertex(b).pnt);
            }
        }

        //      //ArrayList<Point3d> newPoints = new ArrayList<Point3d>(points.size());  
        //      for (Point3d p : points)
        //      {
        //         PointSkinAttachment a = new PointSkinAttachment();
        //         a.setBasePosition (p);
        //         surface.addAttachment (a);
        //         
        //	 //newPoints.add(a.getBasePosition ());
        //      }
    }


    public void setSurface(SkinMeshBody surface)
    {
        this.surface = surface;
    }

}
