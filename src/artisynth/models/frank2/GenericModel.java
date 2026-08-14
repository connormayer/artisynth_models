package artisynth.models.frank2;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.Iterator;
import java.util.List;

import javax.swing.JSeparator;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Vertex3d;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.geometry.Face;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.PlyReader;
import maspack.geometry.io.StlReader;
import maspack.geometry.io.VtkAsciiReader;
import maspack.geometry.io.WavefrontReader;
import maspack.geometry.PolygonalMesh;
import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.properties.Property;
import maspack.properties.PropertyInfo;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderable;
import maspack.util.ReaderTokenizer;
import maspack.widgets.DoubleFieldSlider;
import maspack.widgets.LabeledComponentBase;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.AnsysReader;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.VtkInputOutput;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.TetGenReader;
import artisynth.core.femmodels.UCDReader;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.materials.*;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.DynamicComponentBase;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.DynamicAttachmentComp;
import artisynth.core.mechmodels.FrameSpring;
import artisynth.core.mechmodels.Marker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.Model;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.fluid1d.csa.SliceInterpreter_Components;
import artisynth.models.fluid1d.fileIO.CSV;
import artisynth.models.fluid1d.fileIO.VTK_IO;


public class GenericModel extends RootModel
{
    protected MechModel mechModel = new MechModel();

    protected String meshDir;
    protected String dataDir;
    protected boolean showVisibilityPanel = true;

    // global physical properties
    protected double frictionCoeff = 0.0;

    protected double rbDensity = 1.0;
    protected double femDensity = 1.0;
    protected FemMaterial femMaterial;
    protected IncompMethod femIncompMethod = IncompMethod.AUTO;
    protected double femParticleDamping = 0.0;
    protected double femStiffnessDamping = 0.0;
    protected RenderProps femRendering;
    protected RenderProps rbRendering;

    public GenericModel (String name) throws IOException 
    {
        addModel(mechModel);
        //      meshDir = ArtisynthPath.getSrcRelativePath ( this, "geometry/");
        //      dataDir = ArtisynthPath.getSrcRelativePath ( this, "data/");
    }

    // --- Mesh Readers --- //
    public FemModel3d loadFEM(String name, String meshDir, String meshName, double scale, Color color)
    {
        FemModel3d fem = loadFemMesh(new FemModel3d(), meshDir, meshName);
        fem.setName(name);

        fem.setDensity(femDensity);
        if (femMaterial != null)
            fem.setMaterial(femMaterial.clone());

        // set rendering
        fem.setSurfaceRendering(artisynth.core.femmodels.FemModel.SurfaceRender.Shaded);
        if (femRendering != null)
            fem.setRenderProps(femRendering.clone());
        fem.getRenderProps().setFaceColor(color);

        // finalize
        mechModel.addModel(fem);
        return fem;
    }

    public RigidBody loadRB(String name, String meshDir, String meshName, double scale, Color color)
    {
        RigidBody rb = loadAndAddRigidBody(name, meshDir, meshName, mechModel);
        rb.setName(name);

        rb.setDensity(rbDensity);

        // set rendering
        if (rbRendering != null)
            rb.setRenderProps(rbRendering.clone());
        rb.getRenderProps().setFaceColor(color);

        mechModel.addRigidBody(rb);
        return rb;
    }

    public static boolean loadFemMesh_Tetgen(FemModel3d fem, String meshDir, String meshBasename)
    {
        try 
        {
            String nodeString = meshDir + meshBasename + ".node";
            String elemString = meshDir + meshBasename + ".ele";
            TetGenReader.read ( fem, 1.0, nodeString, elemString, new Vector3d(1.0,1.0,1.0));
            return true;
        }
        catch (IOException e) 
        {
            //e.printStackTrace();
            return false;
        }
    }
    public static boolean loadFemMesh_Ansys(FemModel3d fem, String meshDir, String meshBasename)
    {
        try 
        {
            String nodeString = meshDir + meshBasename + ".node";
            String elemString = meshDir + meshBasename + ".elem";
            AnsysReader.read ( fem, nodeString, elemString, 1.0, new Vector3d(1.0,1.0,1.0), /*options=*/0);
            return true;
        }
        catch (IOException e) 
        {
            //e.printStackTrace();
            return false;
        }
    }
    public static boolean loadFemMesh_UCD(FemModel3d fem, String meshDir, String meshBasename)
    {
        try 
        {
            String filename = meshDir + meshBasename + ".inp";
            UCDReader.read(fem, filename, 1.0, new Vector3d(1.0,1.0,1.0));
            return true;
        }
        catch (IOException e) 
        {
            //e.printStackTrace();
            return false;
        }
    }
    public static boolean loadFemMesh_VTK(FemModel3d fem, String meshDir, String meshBasename)
    {
        String filename = meshDir + meshBasename + ".vtk";
        VtkInputOutput.readUnstructuredMesh_volume(fem, filename);
        return true;
    }
    
    public static FemModel3d loadFemMesh(FemModel3d fem, String meshDir, String meshBasename)
    {
        // a brute-force attempt to read FEM meshes
        if      ( loadFemMesh_VTK(fem, meshDir, meshBasename) == true)
            ;
        else if ( loadFemMesh_Tetgen(fem, meshDir, meshBasename) == true)
            ;
        else if ( loadFemMesh_Ansys(fem, meshDir, meshBasename) == true)
            ;
        else if ( loadFemMesh_UCD(fem, meshDir, meshBasename) == true)
            ;
        else
            System.out.println("Failed to load " + meshBasename);

        return fem;
    }
    
    public static void findDisconnectedNodes(FemModel3d fem, boolean reportNodes, boolean removeNodes)
    {
        // it may be unsafe to remove nodes in the "for" loop, so first build a list of disconnected nodes, then optionally remove
        ArrayList<FemNode3d> dNodes = new ArrayList<FemNode3d>();
        for (FemNode3d node : fem.getNodes () )
            if (node.getElementDependencies ().size () == 0)
                dNodes.add (node);
        
        if (reportNodes == true)
            for (FemNode3d node : dNodes)
                System.out.printf ("%s: node %d is not connected to any elements.\n", fem.getName(), node.myNumber);
        
        if (removeNodes == true)
            for (FemNode3d node : dNodes)
                fem.removeNode (node);
    }

    public static RigidBody loadRigidBody(String name, String meshDir, String meshName)
    {
        PolygonalMesh geom = loadGeometry(meshDir, meshName);
        if (geom == null)
            return null;
        else
        {
            RigidBody rb = new RigidBody();
            rb.setName(name);
            rb.setMesh(geom, null);
            return rb;
        }
    }
    
    public static RigidBody loadAndAddRigidBody(String name, String meshDir, String meshName, MechModel mechModel)
    {
        RigidBody rb = loadRigidBody(name, meshDir, meshName);
        if (rb != null)
            mechModel.addRigidBody(rb);
        return rb;
    }
    
    public static RigidBody createRigidBody(String name, PolygonalMesh mesh, MechModel mechModel)
    {
        RigidBody rb = new RigidBody();
        rb.setName (name);
        rb.setMesh(mesh, null);
        mechModel.addRigidBody (rb);
        return rb;
    }

    public static PolygonalMesh loadGeometry_VTK(String filename)
    {
        try
        {
            PolygonalMesh mesh = VtkAsciiReader.read(filename);
            return mesh;
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("Failed to read geometry: " + filename);
            return null;
        }
    }
    
    public static PolygonalMesh loadGeometry_OBJ(String filename)
    {
        try
        {
            PolygonalMesh mesh = new PolygonalMesh(new File(filename)); // obj file format
            return mesh;
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("Failed to read geometry: " + filename);
            return null;
        }
    }
    
    public static PolygonalMesh loadGeometry(String meshDir, String meshName)
    {
        try
        {
            PolygonalMesh mesh;
            String meshNameLower = meshName.toLowerCase();
            if      (meshNameLower.endsWith(".vtk") == true)
                mesh = VtkAsciiReader.read(meshDir + meshName);
            else
                mesh = (PolygonalMesh)GenericMeshReader.readMesh (meshDir + meshName);
                
            //else if (meshNameLower.endsWith(".obj") == true)
            //    mesh = (PolygonalMesh)WavefrontReader.read(meshDir + meshName);
            //else if (meshNameLower.endsWith(".ply") == true)
            //    mesh = (PolygonalMesh)PlyReader.read(meshDir + meshName);
            //else if (meshNameLower.endsWith(".stl") == true)
            //    mesh = (PolygonalMesh)StlReader.read(meshDir + meshName);
            return mesh;
        }
        catch (Exception e)
        {
            e.printStackTrace();
            System.out.println("Failed to read geometry: " + meshName);
            return null;
        }
    }
    
    public static void reportFemElements(FemModel3d fem)
    {
        int tets = 0;
        int hexes = 0;
        int wedges = 0;
        int pyramids = 0;
        int invTets = 0;
        int invHexes = 0;
        int invWedges = 0;
        int invPyramids = 0;
        
        fem.updateVolume();
        for (FemElement3d elem : fem.getElements() )
        {
            boolean isInverted = false;
            if ( (elem.isInverted() == true) || (elem.getVolume() < 0.0) )
            {
                System.out.printf("%s: elem %d is inverted! Volume = %f. Type: %s \n", fem.getName(), elem.myNumber, elem.getVolume(), elem.getClass().toString() );
                isInverted = true;
            }
            
            if      (elem instanceof TetElement)
            {
                tets++;
                if (isInverted == true)
                    invTets++;
            }
            else if (elem instanceof HexElement)
            {
                hexes++;
                if (isInverted == true)
                    invHexes++;
            }
            else if (elem instanceof WedgeElement)
            {
                wedges++;
                if (isInverted == true)
                    invWedges++;
            }
            else if (elem instanceof PyramidElement)
            {
                pyramids++;
                if (isInverted == true)
                    invPyramids++;
            }
        }
        System.out.printf("FEM model %s contains: %d nodes; %d tets (%d inv), %d hexes (%d inv), %d wedges (%d inv), %d pyramids (%d inv)\n", 
            fem.getName(), fem.numNodes(), tets, invTets, hexes, invHexes, wedges, invWedges, pyramids, invPyramids);
        
    }
    
    public static int[] readIntList(String filename)
    {
        int[] list;
        try 
        {
            ArrayList<Integer> indices = new ArrayList<Integer> ();
            ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (filename));
            rtok.commentChar('#');
            while (rtok.nextToken () != ReaderTokenizer.TT_EOF) 
                indices.add ((int)rtok.lval);
            
            list = new int[indices.size()];
            for (int i=0; i<indices.size(); i++)
                list[i] = indices.get(i);
        }
        catch (IOException e) 
        {
            //e.printStackTrace();
            System.out.println("Failed to read integer list: " + filename);
            list = null;
        }
        return list;
    }
    
    public static void writeIntList(String filename, int[] list)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
            file.print("# List of integers. May contain spaces, line-breaks, and comments.\n\n");
            for (int i : list)
                file.print(i + " ");
            file.println();
            file.close();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    // --- End Mesh Readers --- //

    
    // --- attachments --- //
    
    public static ArrayList<FemNode3d> findNodesNearPoints(FemModel3d fem, ArrayList<Point3d> points, double distance, boolean surfaceNodesOnly)
    {
        //TODO: This is brute force approach, could be optimized.
        
        ArrayList<FemNode3d> nearNodes = new ArrayList<FemNode3d>();

        for(FemNode3d node: fem.getNodes())
        {
            if ( (surfaceNodesOnly == true) && (fem.isSurfaceNode(node) == false) )
                continue;//

            for (Point3d p : points)
                if (p.distance(node.getPosition()) < distance )
                    nearNodes.add (node);
        }

        return nearNodes;
    }
    
    /*/
    public static ArrayList<FemNode3d> findNodesNearPointsBV(FemModel3d fem, ArrayList<Point3d> points, double distance, boolean surfaceNodesOnly)
    {
        ArrayList<FemNode3d> nearNodes = new ArrayList<FemNode3d>();
        
        BVTree bvtree = fem.getBVTree();
        ArrayList<BVNode> bvNodes = new ArrayList<>();
        
        for (Point3d p : points)
        {
            bvNodes.clear();
            bvtree.intersectSphere(bvNodes, p, distance);
            
            fem.findContainingElement(pnt)
            ArrayList<FemNode3d> femNodes; // find femNodes within bvNodes??
            
            for (FemNode3d node : femNodes)
            {
                if ( (surfaceNodesOnly == true) && (fem.isSurfaceNode(node) == false) )
                    ;//
                else
                {
                    if (p.distance(node.getPosition()) < distance )
                        nearNodes.add (node);
                }
            }
            
        }

        return nearNodes;
    }
    //*/
    
    public static ArrayList<FemNode3d> findNodesNearSurface(FemModel3d fem, PolygonalMesh surface, double distance, boolean surfaceNodesOnly)
    {
        ArrayList<FemNode3d> nearNodes = new ArrayList<FemNode3d>();
        
        BVFeatureQuery query = new BVFeatureQuery();
        Point3d nearPnt = new Point3d();
        
        //TODO: perhaps this loop and findNodesInRegion could be optimized by first reducing to nodes in BB
//        BVTree bvtree = fem.getBVTree(); // this needs to be made public
//        ArrayList<BVNode> nodes1 = new ArrayList<>();
//        ArrayList<BVNode> nodes2 = new ArrayList<>();
//        bvtree.intersectTree(nodes1, nodes2, surface.getBVTree());
//        for (BVNode node : nodes1)
//        {
//            // find elements, and gather all nodes from elements
//            // iterate through those nodes rather than all nodes
//        }
        
        for(FemNode3d node: fem.getNodes())
        {
            if ( (surfaceNodesOnly == true) && (fem.isSurfaceNode(node) == false) )
                continue;//
            
            Face nearFace = query.nearestFaceToPoint (nearPnt, null, surface, node.getPosition());
            if (node.getPosition().distance (nearPnt) < distance)
                nearNodes.add (node);
        }
        
        return nearNodes;
    }
    
    public static ArrayList<FemNode3d> findNodesInRegion(FemModel3d fem, PolygonalMesh region, boolean surfaceNodesOnly)
    {
        return findNodesInRegion (fem, region, 0.0, surfaceNodesOnly);
    }
    
    public static ArrayList<FemNode3d> findNodesInRegion(FemModel3d fem, PolygonalMesh region, double tolerance, boolean surfaceNodesOnly)
    {
        ArrayList<FemNode3d> insideNodes = new ArrayList<FemNode3d>();
        
        BVFeatureQuery query = new BVFeatureQuery();
        for(FemNode3d node: fem.getNodes())
        {
            if ( (surfaceNodesOnly == true) && (fem.isSurfaceNode(node) == false) )
                continue;//
            if( query.isInsideMesh (region, node.getPosition(), tolerance) == InsideQuery.INSIDE )
                insideNodes.add (node);
        }
        
        return insideNodes;
    }
    
    public static ArrayList<FemElement3d> findElementsInRegion(FemModel3d fem, PolygonalMesh region)
    {
        return findElementsInRegion(fem, region, 0.0);
    }
    
    public static ArrayList<FemElement3d> findElementsInRegion(FemModel3d fem, PolygonalMesh region, double tolerance)
    {
        ArrayList<FemElement3d> insideElems = new ArrayList<FemElement3d>();
        
        BVFeatureQuery query = new BVFeatureQuery();
        for(FemElement3d elem: fem.getElements())
        {
            Point3d centroid = new Point3d();
            elem.computeCentroid(centroid);
            if( query.isInsideMesh (region, centroid, tolerance) == InsideQuery.INSIDE )
                insideElems.add (elem);
        }
        
        return insideElems;
    }
    
    public static void attachFemNodesToFem(FemModel3d fem1, int[] fem1NodeIndices, FemModel3d fem2, MechModel mechModel)
    {
        attachFemNodesToFem(fem1, getNodesFromIndices(fem1, fem1NodeIndices), fem2, mechModel);
    }
    
    public static void attachFemNodesToFem(FemModel3d fem1, ArrayList<FemNode3d> fem1Nodes, FemModel3d fem2, MechModel mechModel)
    {
        // FEM --> FEM attachment: the node is moved to the FEM surface. Unless the rest position is reset, stress is added.
        Point3d orig_pos = new Point3d();
        boolean inverted = false;

        for(FemNode3d node: fem1Nodes) 
        {
            if( node.isAttached() == false )
            {
                inverted = false;
                orig_pos.set( node.getPosition() );

                //System.out.println ("attaching node " + node.getNumber());
                mechModel.attachPoint(node, fem2);
                //node.setRestPosition(node.getPosition()); // this may be desired in some cases.

                // Reverse node attachment if results in inverted elements
                for(FemElement3d el: node.getElementDependencies()) 
                {
                    if( el.computeVolumes() < 0.0 ) 
                    {
                        inverted = true;
                        System.out.printf ("Warning: attaching %s node %d to %s results in inverted element.\n", fem1.getName(), node.getNumber(), fem2.getName() );
                    }
                }
                if( inverted == true ) 
                {
                    mechModel.detachPoint(node);
                    node.setPosition (orig_pos);
                    for(FemElement3d el: node.getElementDependencies ()) 
                    {
                        if( el.computeVolumes() < 0.0 )
                        {
                            System.out.println ("Warning: inverted element " + el.getNumber());
                        }
                    }
                }
            }
            else
                System.out.printf ("Warning: %s node %d not attached to %s because node already attached.\n", fem1.getName(), node.getNumber(), fem2.getName() );
        }
        fem1.resetRestPosition ();
    }
    
    public static void attachFemNodesToRigidBody(FemModel3d fem1, String fem1IndicesFilename, RigidBody rb, MechModel mechModel)
    {
        attachFemNodesToRigidBody(fem1, getNodesFromIndices(fem1, readIntList(fem1IndicesFilename)), rb, mechModel);
    }
    
    public static void attachFemNodesToRigidBody(FemModel3d fem1, int[] fem1NodeIndices, RigidBody rb, MechModel mechModel)
    {
        attachFemNodesToRigidBody(fem1, getNodesFromIndices(fem1, fem1NodeIndices), rb, mechModel);
    }
    
    public static void attachFemNodesToRigidBody(FemModel3d fem1, ArrayList<FemNode3d> fem1Nodes, RigidBody rb, MechModel mechModel)
    {
        // FEM --> RB attachment: the node is not moved, hence no stress is introduced to the FEM
        for (FemNode3d node : fem1Nodes)
            if( node.isAttached() == false )
                mechModel.attachPoint(node, rb);
    }
    
    public static void attachFemToFem(FemModel3d fem1, FemModel3d fem2, double distance, MechModel mechModel)
    {
        ArrayList<FemNode3d> nodesToAttach = findNodesNearSurface(fem1, fem2.getSurfaceMesh(), distance, true);
//        System.out.println (
//           "fem-fem-attach: " + fem1.getName()+"-"+fem2.getName());
//        System.out.println ("num nodes " + nodesToAttach.size());
        attachFemNodesToFem(fem1, nodesToAttach, fem2, mechModel);
//        System.out.println ("done");
    }
    
    /**
     * @param fem1 - the nodes of fem1 are attached to fem2
     * @param fem2 - fem1 is attached to fem2
     * @param attachmentSurface - fem1 and fem2 are only connected where fem1 is within "distance" of attachment surface
     * @param distance - the tolerance distance between fem1 and the attachment surface
     * @param mechModel
     */
    public static void attachFemToFem(FemModel3d fem1, FemModel3d fem2, PolygonalMesh attachmentSurface, double distance, MechModel mechModel)
    {
        ArrayList<FemNode3d> fem1Nodes =  findNodesNearSurface(fem1, attachmentSurface, distance, true);
        attachFemNodesToFem(fem1, fem1Nodes, fem2, mechModel);
    }
    
    /**
     * @param fem - the nodes of fem are attached to rb
     * @param rb - fem is attached to rb
     * @param attachmentSurface - fem and rb are only connected where fem is within "distance" of attachment surface
     * @param distance - the tolerance distance between fem and the attachment surface
     * @param mechModel
     */
    public static void attachFemToRigidBody(FemModel3d fem, RigidBody rb, PolygonalMesh attachmentSurface, double distance, MechModel mechModel) 
    {
        ArrayList<FemNode3d> femNodes =  findNodesNearSurface(fem, attachmentSurface, distance, true);
        attachFemNodesToRigidBody(fem, femNodes, rb, mechModel);
    }
    
    public static void attachFemToRigidBody(FemModel3d fem, RigidBody rb, double distance, boolean surfaceOnly, MechModel mechModel) 
    {
        ArrayList<FemNode3d> nodesToAttach;
        if (surfaceOnly == true)
            nodesToAttach = findNodesNearSurface(fem, rb.getSurfaceMesh(), distance, true);
        else
            nodesToAttach = findNodesInRegion(fem, rb.getSurfaceMesh(), distance, false);
        
        attachFemNodesToRigidBody(fem, nodesToAttach, rb, mechModel);
    }
    
    public static void setNodesNondynamic(FemModel3d fem, int[] nodeIndices)
    {
        for (int i=0; i<nodeIndices.length; i++)
            fem.getNode(nodeIndices[i]).setDynamic(false);
    }
    
    public static void setNodesDynamic(FemModel3d fem, int[] nodeIndices)
    {
        for (int i=0; i<nodeIndices.length; i++)
            fem.getNode(nodeIndices[i]).setDynamic(true);
    }
    
    public static void setNodesNondynamic(ArrayList<FemNode3d> nodes)
    {
        for (FemNode3d node : nodes)
            node.setDynamic(false);
    }
    
    public static ArrayList<FemNode3d> getNodesFromIndices(FemModel3d fem, int[] nodeIndices)
    {
        ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
        for (int i=0; i<nodeIndices.length; i++)
            nodes.add(fem.getNode(nodeIndices[i]) );
        return nodes;
    }
    
    public static int[] getIndicesFromNodes(ArrayList<FemNode3d> nodes)
    {
        int[] list = new int[nodes.size()];
        for (int i=0; i<nodes.size(); i++)
            list[i] = nodes.get(i).myNumber;
        return list;
    }

    //    protected PolygonalMesh findMatchingSurface(PolygonalMesh geometry, PolygonalMesh appxSurface)
    //    {
    //        // find the nearest surface nearest to the input surface. TODO: not working
    //
    //        PolygonalMesh surface = new PolygonalMesh();
    //        //surface.add
    //        ArrayList<Face> fsiFaces = new ArrayList<Face>();
    //
    //        OBBTree obbt = geometry.getObbtree();
    //        Point3d proj = new Point3d();
    //        Vector2d coords = new Vector2d();
    //        Intersector isect = new Intersector();
    //        Point3d centroid = new Point3d();
    //
    //        for (Face face : appxSurface.getFaces())
    //        {
    //            face.computeWorldCentroid(centroid);
    //            Face nearestFace = obbt.nearestFace(centroid, null, proj, coords, isect);
    //            if (fsiFaces.contains(nearestFace) == false)
    //                fsiFaces.add(nearestFace);
    //        }
    //
    //        return surface;
    //    }

    public static FrameSpring makeFrameSpring (String name, RigidBody bodyA, RigidBody bodyB, double x, double y, double z) 
    {
        RigidTransform3d XDW = new RigidTransform3d();
        RigidTransform3d XCA = new RigidTransform3d();
        RigidTransform3d XDB = new RigidTransform3d();
        XDW.p.set (x, y, z);

        XCA.mulInverseLeft (bodyA.getPose(), XDW);
        XDB.mulInverseLeft (bodyB.getPose(), XDW);

        FrameSpring spring = new FrameSpring (null);
        spring.setName(name);
        spring.setFrameA(bodyA);
        spring.setFrameB(bodyB);
        spring.setAttachFrameA (XCA);
        spring.setAttachFrameB (XDB);
        
        return spring;
     }
    
    // --- End Attachments --- //
    
    // --- Probes --- //

    /*//
    public static NumericInputProbe createMuscleProbe(FemMuscleModel fem, String name, double[] time, double[] activation)
    {
        NumericInputProbe probe = new NumericInputProbe();
        probe.setName(name);

        Interpolation interp = new Interpolation();
        interp.setOrder(Order.Linear); // TODO: make this an option...
        probe.setInterpolation(interp);

        probe.setStartStopTimes(0.0, time[time.length-1]);   // time-axis
        probe.setDefaultDisplayRange(0.0, 1.0);              // y-axis
        probe.setModel(fem);

        Property[] props = {fem.getMuscleBundles().get(name).getProperty("excitation")};
        String[] driverExpressions = {"V0"};  // ??
        String[] variableNames = {"V0"};      // ??
        int[] variableDimensions = {1};
        probe.set(props, driverExpressions, variableNames, variableDimensions, null);

        for (int a=0; a<time.length; a++)
        {
            probe.addData(new double[] {time[a], activation[a]}, NumericInputProbe.EXPLICIT_TIME);
        }

        return probe;
    }
    //*/
    
    // --- End Probes --- //
    
    public static void createDirectory(String dir)
    {
        File fDir = new File(dir);
        if (fDir.exists() == false)
            fDir.mkdirs();
            //fDir.mkdir();
    }

    public static ArrayList<FemModel3d> getAllFemModels(MechModel mm)
    {
        ArrayList<FemModel3d> fems = new ArrayList<FemModel3d>();
         
        ArrayList<MechModel> mms = getAllMechModels(mm);
        for (MechModel mechModel : mms)
            for (MechSystemModel msm : mechModel.models())
                if (msm instanceof FemModel3d)
                    fems.add( (FemModel3d)msm );
        return fems;
    }
    
    public static ArrayList<FemMuscleModel> getAllFemMuscleModels(MechModel mm)
    {
        ArrayList<FemModel3d> fems = getAllFemModels(mm);
        ArrayList<FemMuscleModel> fmms = new ArrayList<FemMuscleModel>(); 
         
        for (FemModel3d fem : fems)
            if (fem instanceof FemMuscleModel)
                fmms.add ((FemMuscleModel)(fem));
        
        return fmms;
    }
    
    public static ArrayList<MechModel> getAllMechModels(MechModel mm)
    {
        // recursive function to locate all MechModels for the case that MechModels are embedded within the main one 
        ArrayList<MechModel> mms = new ArrayList<MechModel>();
        mms.add(mm);
        for (MechSystemModel msm : mm.models())
            if (msm instanceof MechModel)
                mms.addAll ( getAllMechModels((MechModel)msm) );

        return mms;
    }

    public static void writeAllGeometries(String folder, MechModel mechModel)
    {
        createDirectory(folder);
        
        ArrayList<MechModel> mechModels = getAllMechModels(mechModel); // this get clunky because the mechModel might have contain other mechModels...
        for (MechModel mm : mechModels)
        {
            writeAxialSprings (folder, mm);
            for (MechSystemModel msm : mm.models())
            {
                if (msm instanceof FemModel3d)
                {
                    FemModel3d model = (FemModel3d)msm;
                    try
                    {
                        //GenericMeshWriter.writeMesh (folder + model.getName() + ".stl", model.getSurfaceMesh() );
                        //findDisconnectedNodes(model, false, true);
                        VTK_IO.writeVTK(folder + model.getName() + ".vtk", model);
                        VTK_IO.writeVTK(folder + model.getName() + "_surface.vtk", model.getSurfaceMesh() );
                    }
                    catch(Exception e){}
                    
                    if (model instanceof FemMuscleModel)
                    {
                        FemMuscleModel fmm = (FemMuscleModel)model;
                        writeMuscleBundles(folder, fmm);
                    }
                    
                    try
                    {
                        String filename = folder + model.getName() + "_staticnodes.txt";
                        PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
                        for (FemNode3d node : model.getNodes())
                            if (node.isDynamic() == false)
                                file.printf("%d,", node.myNumber);
                        file.close();
                    }
                    catch (Exception e)
                    {
                    }
                    
                    try
                    {
                        String filename = folder + model.getName() + "_attachednodes.txt";
                        PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
                        for (FemNode3d node : model.getNodes())
                            if (node.isAttached() == true)
                                file.printf("%d,", node.myNumber);
                        file.println("");
                        
                        file.println("details...");
                        for (FemNode3d node : model.getNodes()) {
                           DynamicAttachment at = node.getAttachment();
                           if (at instanceof DynamicAttachmentComp)
                            {
                               DynamicAttachmentComp ac =
                                  (DynamicAttachmentComp)at;
                               file.printf("%d: ", node.myNumber);
                               ArrayList<ModelComponent> refs =
                                  new ArrayList<ModelComponent>();
                               ac.getHardReferences (refs);
                               for (ModelComponent mc : refs)
                                  file.printf("%s, ", mc.getName() );
                               file.println();
                            }
                        }
                        file.close();
                    }
                    catch (Exception e)
                    {
                    }
                }
            }
            for (RigidBody rb : mm.rigidBodies ())
            {
                try
                {
                    PolygonalMesh mesh = rb.getSurfaceMesh();
                    RigidTransform3d trans = mesh.getMeshToWorld();
                    mesh.transform(trans);
                    mesh.setMeshToWorld (new RigidTransform3d());

                    GenericMeshWriter.writeMesh (folder + rb.getName() + ".stl", rb.getSurfaceMesh() );
                    VTK_IO.writeVTK (folder + rb.getName() + ".vtk", rb.getSurfaceMesh() );
                }
                catch(Exception e){}
                
                //rb.getFrameMarkers ();
            }
            //mm.addAxialSpring(s)
        }

    }
    
    public static void writeAxialSprings(String folder, MechModel mechModel)
    {
        // this should be very similar to writing muscle bundles...combine them?
        try
        {
            String filename = folder + mechModel.getName() + "_axialSprings.txt";
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
            
            for (AxialSpring as : mechModel.axialSprings())
            {   
                Point[] points = {as.getFirstPoint(), as.getSecondPoint()};
                AxialMaterial mat = as.getMaterial();
                file.printf("spring name: %s\n", as.getName() );
                file.printf("material: %s, %f, %f\n", mat.getClass(), as.getLength(), as.getRestLength());
                for (Point p : points)
                {
                    file.printf("%s, %f, %f, %f, %s, %b, %b", p.getName(), p.getPosition().x, p.getPosition().y, p.getPosition().z,
                        p.getClass().toString(), p.isDynamic(), p.isAttached());
                    DynamicAttachment at = p.getAttachment();
                    if (at instanceof DynamicAttachmentComp) {
                       DynamicAttachmentComp ac =
                          (DynamicAttachmentComp)at;
                        ArrayList<ModelComponent> refs =
                           new ArrayList<ModelComponent>();
                        ac.getHardReferences(refs );
                        for (ModelComponent mc : refs)
                            file.printf(", %s", mc.getName() );
                        file.println();
                    }
                }
                
            }
            file.println();
            
            file.println("In code:");
            for (AxialSpring as : mechModel.axialSprings())
            {   
                Point p1 = as.getFirstPoint();
                Point p2 = as.getSecondPoint();
                //as.get
                AxialMaterial mat = as.getMaterial();
                double length = as.getLength();
                file.printf("addAxialSpring(mechModel, \"%s\", ", as.getName() );
                if (mat instanceof PeckAxialMuscle)
                {
                    //AxialMuscleMaterial pMat = (AxialMuscleMaterial)mat;
                    PeckAxialMuscle pMat = (PeckAxialMuscle)mat;
                    file.printf("peckMuscle(%f, %f, %f, %f, %f, %f, %f),\n", pMat.getDamping(), pMat.getMaxForce(), pMat.getPassiveFraction(), pMat.getOptLength()/length, pMat.getMaxLength()/length, pMat.getTendonRatio(), pMat.getForceScaling() );
                }
                else if (mat instanceof LinearAxialMuscle)
                {
                    LinearAxialMuscle lMat = (LinearAxialMuscle)mat; 
                    file.printf("peckMuscle(%f, %f, %f, %f, %f, %f, %f),\n", lMat.getDamping(), lMat.getMaxForce(), lMat.getPassiveFraction(), lMat.getOptLength()/length, lMat.getMaxLength()/length, lMat.getTendonRatio(), lMat.getForceScaling() );
                }
                else if (mat instanceof LinearAxialMaterial)
                {
                    LinearAxialMaterial lMat = (LinearAxialMaterial)mat; 
                    file.printf("linearMaterial(%f, %f),\n", lMat.getStiffness(), lMat.getDamping());
                }
                else
                {
                    file.printf("null,\n");
                }
                ArrayList<ModelComponent> refs1 = new ArrayList<ModelComponent>();
                DynamicAttachment at = p1.getAttachment();
                if (at instanceof DynamicAttachmentComp) {
                   ((DynamicAttachmentComp)at).getHardReferences(refs1);
                   file.printf("%f, %f, %f, %s,\n",    p1.getPosition().x, p1.getPosition().y, p1.getPosition().z, refs1.get(1).getName() );
                }
                ArrayList<ModelComponent> refs2 = new ArrayList<ModelComponent>();
                at = p2.getAttachment();
                if (at instanceof DynamicAttachmentComp) {
                   ((DynamicAttachmentComp)at).getHardReferences(refs2);
                   file.printf("%f, %f, %f, %s);\n\n", p2.getPosition().x, p2.getPosition().y, p2.getPosition().z, refs2.get(1).getName() );
                }
                
            }
            
            file.close();
        }
        catch(Exception e)
        {
        }
    }
    
    public static void writeMuscleBundles(String folder, FemMuscleModel model)
    {
        for (MuscleBundle mb : model.getMuscleBundles())
        {
            // write muscle bundle material props
            // write muscle bundle geoms...
            try
            {
                String filename = folder + model.getName() + "_bundle_" + mb.getName() + ".txt";
                PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
                ArrayList<Point> points = new ArrayList<Point>();
                
                for (Muscle m : mb.getFibres())
                {
                    if ( points.contains(m.getFirstPoint()) == false )
                        points.add (m.getFirstPoint ());
                    if ( points.contains(m.getSecondPoint()) == false )
                        points.add (m.getSecondPoint ());
                        
                    
//                    Point p1 = m.getFirstPoint();
//                    Point p2 = m.getSecondPoint();
//                    file.printf("%s,%f,%f,%f,%s,%f,%f,%f\n", 
//                        p1.toString(), p1.getPosition().x, p1.getPosition().y, p1.getPosition().z, 
//                        p2.toString(), p2.getPosition().x, p2.getPosition().y, p2.getPosition().z);
//                    m.getForce();   // I think this should depend on activations/gesture...not a generic prop
//                    m.getPassiveForce();    //
//                    m.getMaterial();
//                    m.getRestLength();
//                    m.getMaxForce(s);
                }
                file.printf("muscle bundle: %s\n\n", mb.getName() );
                file.println("points");
                file.println("index,x,y,z,class,dynamic,active,fixed,attached");
                for (int i=0; i<points.size (); i++)
                {
                    Point p = points.get (i);
                    file.printf("%d,%f,%f,%f,%s,%b,%b,%b,%b\n", i, 
                        p.getPosition().x, p.getPosition().y, p.getPosition().z, p.getClass().toString(),
                        p.isDynamic(), p.isActive(), p.isFixed (), p.isAttached());
                }
                file.println();
                
                file.println("points in code");
                file.printf("public static double[][] points_%s = {", mb.getName() );
                for (int i=0; i<points.size (); i++)
                {
                    Point p = points.get (i);
                    
                    file.printf("\n\t{ %+013.8f, %+013.8f, %+013.8f }, /* %04d */", p.getPosition().x, p.getPosition().y, p.getPosition().z, i);
                }
                file.printf("};\n\n");
                
                //file.println("connections");
                int lastIndex = -1;
                file.printf("public static int[][] connections_%s = {\n", mb.getName() );
                for (Muscle m : mb.getFibres())
                {
                    int i1 = points.indexOf(m.getFirstPoint());
                    int i2 = points.indexOf(m.getSecondPoint());
                    if (i1 != lastIndex)
                    {
                        if (lastIndex == -1)
                            ;
                        else
                            file.printf( "},\n" );
                        file.printf("\t{%d, %d", i1, i2 );
                    }
                    else
                    {
                        file.printf(", %d", i2 );
                    }
                    lastIndex = i2;
                }
                file.printf( "}};\n" );
                file.println();
                
                file.println("material");
                
                for (Muscle m : mb.getFibres())
                {
                    AxialMaterial mat = m.getMaterial();
                    double length = m.getLength();
                    if (mat == null)
                        file.print ("null material");
                    else if (mat instanceof PeckAxialMuscle)
                    {
                        //AxialMuscleMaterial pMat = (AxialMuscleMaterial)mat;
                        PeckAxialMuscle pMat = (PeckAxialMuscle)mat;
                        file.printf("peckMuscle(%f, %f, %f, %f, %f, %f, %f),\n", pMat.getDamping(), pMat.getMaxForce(), pMat.getPassiveFraction(), pMat.getOptLength()/length, pMat.getMaxLength()/length, pMat.getTendonRatio(), pMat.getForceScaling() );
                    }
                    else if (mat instanceof ConstantAxialMuscle)
                    {
                        ConstantAxialMuscle pMat = (ConstantAxialMuscle)mat;
                        file.printf("constantMuscle(%f, %f, %f, %f, %f, %f, %f),\n", pMat.getDamping(), pMat.getMaxForce(), pMat.getPassiveFraction(), pMat.getOptLength()/length, pMat.getMaxLength()/length, pMat.getTendonRatio(), pMat.getForceScaling() );
                    }
                    // constant axial muscle
                    else
                    {
                        file.printf("%s: ", mat.getClass().toString());
                        for (PropertyInfo info : mat.getAllPropertyInfo() )
                            file.printf("%s=%s;", info.getName(), mat.getProperty(info.getName()).toString() );
                        file.println();
                    }
                }
                
                /*//
                file.println("points in code");
                file.println("x,y,z,snapToFem,snapToFemTol,snapToNearestNode,snapToNearestNodeTol");
                file.printf("mb.setName( \"%s\" );\n", mb.getName() );
                for (int i=0; i<points.size (); i++)
                {
                    Point p = points.get (i);
                    
                    boolean snapToFem = true;
                    boolean snapToNearestNode = false;
                    if (p instanceof FemNode3d)
                    {
                        if (p.isDynamic() == false)
                            snapToFem = false;
                        else
                            snapToNearestNode = true;
                    }
                    
                    file.printf("mb.addPoint( %f,%f,%f,%b, snapToFemTol,%b, snapToNearestNodeTol );\n",  
                        p.getPosition().x, p.getPosition().y, p.getPosition().z, snapToFem, snapToNearestNode);
                }
                file.println();
                
                //file.println("connections");
                int lastIndex = -1;
                file.println( "mb.connections = new int[][]{" );
                for (Muscle m : mb.getFibres())
                {
                    int i1 = points.indexOf(m.getFirstPoint());
                    int i2 = points.indexOf(m.getSecondPoint());
                    if (i1 != lastIndex)
                    {
                        if (lastIndex == -1)
                            ;
                        else
                            file.printf( "},\n" );
                        file.printf("{%d, %d", i1, i2 );
                    }
                    else
                    {
                        file.printf(", %d", i2 );
                    }
                    lastIndex = i2;
                }
                file.printf( "}};\n" );
                file.println();
                //*/
                
                file.println("connections: simple way");
                for (Muscle m : mb.getFibres())
                {
                    file.printf("%d,%d\n", points.indexOf(m.getFirstPoint()), points.indexOf(m.getSecondPoint()) );
                }
                file.println();
                
                file.close();
            }
            catch(Exception e)
            {
            }
            
            
        }
    }
    
    public static void writeVerticesToCSV(String filename, ArrayList<Vertex3d> vertices)
    {
        int nPoints = vertices.size();
        double[][] data = new double[nPoints][];
        for (int a=0; a<nPoints; a++)
        {
            Point3d pnt = vertices.get(a).getPosition();
            data[a] = new double[]{pnt.x, pnt.y, pnt.z};
        }
        String[] headers = {"x", "y", "z"};
        CSV.Write(filename, headers, data);
    }
    
    public static void writePointsToCSV(String filename, ArrayList<Point3d> points)
    {
        int nPoints = points.size();
        double[][] data = new double[nPoints][];
        for (int a=0; a<nPoints; a++)
        {
            Point3d pnt = points.get(a);
            data[a] = new double[]{pnt.x, pnt.y, pnt.z};
        }
        String[] headers = {"x", "y", "z"};
        CSV.Write(filename, headers, data);
    }
    
    public static void writeClassState(RootModel rootModel, String filename)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
            
            Date dNow = new Date();
            SimpleDateFormat ft = new SimpleDateFormat ("yyyy.MM.dd'_'HH:mm:ss zzz");
            file.printf("Created on %s. \n\n", ft.format(dNow));  
            
            for(Field f : rootModel.getClass().getDeclaredFields())
            {
               f.setAccessible(true);  // force allowing access
                file.println(f.getType() + " " + f.getName() + " = " + f.get(rootModel) );
            }
            
            /*/ if I want to be more pick about what to print...
            int nFields = rootModel.getClass().getDeclaredFields().length;
            for (int a=0; a<nFields; a++)
            {
                if (rootModel.getClass().getDeclaredFields()[a].getType().isPrimitive() == true)
                {
                    file.println(rootModel.getClass().getDeclaredFields()[a].getName() + " = " + rootModel.getClass().getDeclaredFields()[a].get(rootModel));
                }
                else if (rootModel.getClass().getDeclaredFields()[a].getType().isInstance(String.class) == true )
                {
                    // doesn't work...find nicer way to handle this
                    file.println(rootModel.getClass().getDeclaredFields()[a].getName() + " = " + rootModel.getClass().getDeclaredFields()[a].get(rootModel));
                }
                else if (rootModel.getClass().getDeclaredFields()[a].getType().isEnum() == true)
                {
                    file.println(rootModel.getClass().getDeclaredFields()[a].getName() + " = " + rootModel.getClass().getDeclaredFields()[a].get(rootModel));
                }
            }
            //*/
            file.close();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }
    
    // --- Rendering --- //
    
    public static Color[] createColors_RedBlue()
    {
        Color[] colors = {
                          new Color(1.00f, 0.50f, 0.00f).brighter(), new Color(1.00f, 0.50f, 0.00f), new Color(1.00f, 0.50f, 0.00f).darker(),
                          new Color(1.00f, 0.00f, 0.00f).brighter(), new Color(1.00f, 0.00f, 0.00f), new Color(1.00f, 0.00f, 0.00f).darker(),
                          new Color(1.00f, 0.00f, 0.50f).brighter(), new Color(1.00f, 0.00f, 0.50f), new Color(1.00f, 0.00f, 0.50f).darker(),
                          new Color(1.00f, 0.00f, 1.00f).brighter(), new Color(1.00f, 0.00f, 1.00f), new Color(1.00f, 0.00f, 1.00f).darker(),
                          new Color(0.50f, 0.00f, 1.00f).brighter(), new Color(0.50f, 0.00f, 1.00f), new Color(0.50f, 0.00f, 1.00f).darker(),
                          };
        return colors;
    }
    
    public static Color[] createColors_Reds()
    {
        return createColors_Reds(5);
    }
    
    public static Color[] createColors_Reds(int nHues)
    {   
        // hue: ranging from 270 (violet) --> 45 (orange)
        // saturation
        // brightness/value
        float step = (360.0f-270.0f + 45.0f)/(float)(nHues-1);
        Color[] colors = new Color[3*nHues];
        for (int i=0; i<nHues; i++)
        {
            float hue = (270f + step*i)/360.0f; // I think the mod gets handled
            colors[i*3 + 0] = Color.getHSBColor(hue, 0.5f, 1.0f);
            colors[i*3 + 1] = Color.getHSBColor(hue, 1.0f, 1.0f);
            colors[i*3 + 2] = Color.getHSBColor(hue, 1.0f, 0.5f);
        }
        return colors;
    }
    
    public static void renderPoints(Iterable<? extends Point> points, double size, Color color)
    {
        for (Point point : points)
        {
//            if (point.getRenderProps() == null)
//                point.setRenderProps(new RenderProps());
//            point.getRenderProps().setPointColor(color);
//            point.getRenderProps().setPointRadius(size);
//            point.getRenderProps().setPointStyle(PointStyle.SPHERE);
//            point.getRenderProps().setVisible(true);
            RenderProps.setPointColor(point, color);
            RenderProps.setPointRadius(point, size);
            RenderProps.setPointStyle(point, PointStyle.SPHERE);
            RenderProps.setVisible(point, true);
        }
    }

    public static void renderMarkers(ArrayList<Marker> markers, double size, Color color)
    {
        renderPoints(markers, size, color);
    }

    public static void renderNodes(ArrayList<FemNode3d> nodes, double size, Color color)
    {
        renderPoints(nodes, size, color);
    }
    
    public static void renderFemMarkers(FemModel3d model, double size, Color color)
    {
        ArrayList<Marker> markers = new ArrayList<Marker>( model.markers().size() );
        for (Marker m : model.markers() )
            markers.add(m);
        renderPoints(markers, size, color);
    }
    
    public static void renderFrameMarkers(MechModel mech, double size, Color color)
    {
        ArrayList<Marker> markers = new ArrayList<Marker>( mech.frameMarkers().size() );
        for (Marker m : mech.frameMarkers() )
            markers.add(m);
        renderPoints(markers, size, color);
    }

    public static void renderStaticPoints(Iterable<? extends Point> points, double size, Color color)
    {
        ArrayList<Point> nodes = new ArrayList<Point>();
        for (Point p : points)
            if (p.isDynamic() == false)
                nodes.add(p);
        
        renderPoints(nodes, size, color);
    }
    
    public static void renderAttachedPoints(Iterable<? extends Point> points, double size, Color color)
    {
        ArrayList<Point> nodes = new ArrayList<Point>();
        for (Point p : points)
            if (p.isAttached() == true)
                nodes.add(p);
        
        renderPoints(nodes, size, color);
    }
    
    public static void renderStaticNodes(FemModel3d model, double size, Color color)
    {
        renderStaticPoints(model.getNodes(), size, color);
    }

    public static void renderAttachedNodes(FemModel3d model, double size, Color color)
    {
        renderAttachedPoints(model.getNodes(), size, color);
    }

    // ----- controls -----
    public void attach (DriverInterface driver) 
    {
        if (showVisibilityPanel == true)
        {
            ControlPanel panel = createVisibilityPanel(mechModel);
            addControlPanel(panel);
        }

    }
    
    public static void buildControlPanels(RootModel rootModel, MechModel mechModel, boolean femMuscleControls, boolean muscleBundleControls, boolean visibilityControls)
    {
       ArrayList<MechModel> mechModels = getAllMechModels(mechModel); // this get clunky because the mechModel might have contain other mechModels...
       for (MechModel mm : mechModels)
       {
          for (MechSystemModel msm : mm.models())
          {
             if (msm instanceof FemMuscleModel)
             {
                FemMuscleModel model = (FemMuscleModel)msm;
                if (femMuscleControls == true)
                {
                    ControlPanel generalControls = FemControlPanel.createControlPanel(rootModel, model, mechModel);
                    //rootModel.addControlPanel(generalControls);
                }
                if ( ( muscleBundleControls == true) && (model.getMuscleBundles().size()>0) )
                {
                   ControlPanel muscleControls = FemControlPanel.createMuscleBundlesPanel(rootModel, model);
                   //this.addControlPanel(muscleControls);
                }
             }
          }
       }
       
       if (visibilityControls == true)
       {
           ControlPanel visibleControls = createVisibilityPanel(mechModel);
           rootModel.addControlPanel(visibleControls);
       }
       
       rootModel.mergeAllControlPanels (true);        // one window with all control panels in tabs 
    }
    
    public static void buildFemPanels(RootModel rootModel, MechModel mechModel, ComponentList<FemMuscleModel> fems, boolean muscleBundleControls)
    {
        for (FemMuscleModel model : fems)
        {
            ControlPanel generalControls = FemControlPanel.createControlPanel(rootModel, model, mechModel);
            if ( ( muscleBundleControls == true) && (model.getMuscleBundles().size()>0) )
            {
                ControlPanel muscleControls = FemControlPanel.createMuscleBundlesPanel(rootModel, model);
            }
        }
    }
    
    public static void buildVisibilityPanel(RootModel rootModel, MechModel mechModel) 
    {
        ControlPanel panel = createVisibilityPanel(mechModel);
        rootModel.addControlPanel(panel);
    }
    
    public static ControlPanel initVisibilityPanel(MechModel mechModel) 
    {
        ControlPanel panel = new ControlPanel ("Show", "LiveUpdate");
        //panel.setLocation (Main.getMainFrame().getSize().width, 0);
        panel.pack();
        panel.setVisible(true);
        return panel;
    }
    public static void addGroupToVisibilityPanel(ControlPanel panel, RenderableComponentList<? extends ModelComponent> objs)
    {
        panel.addWidget (new JSeparator());
        if (objs.getRenderProps() == null)
            objs.setRenderProps(new RenderProps()); // Render props for the component list must be initialized
        panel.addWidget (objs.getName()+" (all)", objs, "renderProps.visible");
        //LabeledComponentBase widget = panel.addWidget (objs.getName()+" (all)", objs, "renderProps.visible");
        panel.addWidget (new JSeparator());
        for (ModelComponent obj : objs)
            panel.addWidget (obj.getName(), obj, "renderProps.visible");
        panel.addWidget (new JSeparator());
    }
    
    public static void addComponentsToVisibilityPanel(ControlPanel panel, ComponentList<? extends RenderableComponent> objs)
    {
        panel.addWidget (new JSeparator());
        for (ModelComponent obj : objs)
            panel.addWidget (obj.getName(), obj, "renderProps.visible");
        panel.addWidget (new JSeparator());
    }
    
    public static void addComponentsToVisibilityPanel(ControlPanel panel, ComponentListView<? extends RenderableComponent> objs)
    {
        panel.addWidget (new JSeparator());
        for (ModelComponent obj : objs)
            panel.addWidget (obj.getName(), obj, "renderProps.visible");
        panel.addWidget (new JSeparator());
    }
    
    public static void addComponentsToDynamicsPanel(ControlPanel panel, Iterable<? extends RigidBody> objs)
    {
        // TODO: I would like a general dynamics control for RBs and FEMs. Switching FEM dynamics is more complicated, in which case I may want to add props to the root model...
        panel.addWidget (new JSeparator());
        for (RigidBody obj : objs)
            panel.addWidget (obj.getName(), obj, "dynamic");
        panel.addWidget (new JSeparator());
    }
    
    public static ControlPanel createVisibilityPanel(MechModel mechModel) 
    {
        ControlPanel panel = new ControlPanel ("Show", "LiveUpdate");
        
        panel.addWidget("FrameMarkers", mechModel.frameMarkers(), "renderProps.visible");
        panel.addWidget("AxialSprings", mechModel.axialSprings(), "renderProps.visible");
        panel.addWidget("Particles",    mechModel.particles(),    "renderProps.visible");
        
        panel.addWidget (new JSeparator());
        
        for (RigidBody body : mechModel.rigidBodies()) 
        {
            //if(!body.getName().matches("ref_block"))
            panel.addWidget (body.getName (), body, "renderProps.visible");
        }
        panel.addWidget (new JSeparator());
        
        for (MeshComponent mb : mechModel.meshBodies())
            panel.addWidget (mb.getName (), mb, "renderProps.visible");
        
        panel.addWidget (new JSeparator());
        for (Model mod : mechModel.models()) 
        {
            panel.addWidget (mod.getName(), mod, "renderProps.visible");
        }
        panel.setLocation (Main.getMain().getMainFrame().getSize().width, 0);
        panel.pack();
        panel.setVisible(true);
        
        return panel;
    }
    
    public static ControlPanel createBundleControls(ComponentList<MuscleBundle> bundles) 
    {
        ControlPanel controlPanel = new ControlPanel(bundles.getName(), "LiveUpdate");
        controlPanel.setScrollable(true);
        for (MuscleBundle b : bundles) 
        {
            LabeledComponentBase widget = controlPanel.addWidget(b.getName(), b, "excitation");
            if (b.getRenderProps() != null) {
                widget.setLabelFontColor(b.getRenderProps().getLineColor());
            }
        }
        return controlPanel;
    }
    
    public static ControlPanel createExciterControls(String name, Iterable<MuscleExciter> muscleExciters) 
    {
        ControlPanel panel = new ControlPanel(name, "LiveUpdate");
        panel.setScrollable(true);
        for (MuscleExciter mex : muscleExciters) 
        {
            //DoubleFieldSlider slider = (DoubleFieldSlider)panel.addWidget(mex.getName(), fem, "exciters/" + mex.getNumber() + ":excitation", 0, 1);
            //DoubleFieldSlider slider = (DoubleFieldSlider)panel.addWidget(mex.getName(), mex, "excitation", 0, 1);
            LabeledComponentBase widget = panel.addWidget(mex.getName(), mex, "excitation", 0, 1);
            RenderProps rp = ((Renderable)mex.getTarget(0)).getRenderProps();
            if (rp != null)
                widget.setLabelFontColor( rp.getLineColor());
            //slider.setRoundingTolerance(0.00001);
            //slider.getLabel().setForeground(FemControlPanel.getMuscleExciterColor(mex.getNumber()));
        }
        return panel;
    }
    
    public static ControlPanel createExciterControls_colored(Iterable<MuscleExciter> muscleExciters, Iterable<MuscleBundle> bundles) 
    {
        ControlPanel panel = new ControlPanel("Excitations", "LiveUpdate");
        panel.setScrollable(true);
        
        //muscleExciters.
        Iterator<MuscleExciter> me_it = muscleExciters.iterator();
        Iterator<MuscleBundle> mb_it = bundles.iterator();
        while(me_it.hasNext() && mb_it.hasNext())
        {
            MuscleExciter mex = me_it.next();
            MuscleBundle musc = mb_it.next();
            LabeledComponentBase widget = panel.addWidget(mex.getName(), mex, "excitation", 0, 1);
            if (musc.getRenderProps() != null)
                widget.setLabelFontColor(musc.getRenderProps().getLineColor());
        }
        return panel;
    }
    
    static RenderableComponentList<RenderableComponent> makeComponentList(ComponentListView<? extends RenderableComponent> clv)
    {
        RenderableComponentList<RenderableComponent> cl = new RenderableComponentList<RenderableComponent>(RenderableComponent.class);
        cl.createRenderProps();
        for (RenderableComponent comp : clv)
        {
            cl.add(comp);
        }
        return cl;
    }

}

// functionality to add
// * general fem reader, rigidBody reader
// * general auto-attach --> try to make this fast...not node-by-node
// * 
