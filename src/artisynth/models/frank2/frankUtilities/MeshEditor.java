package artisynth.models.frank2.frankUtilities;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemNodeNeighbor;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.VtkInputOutput;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.mechmodels.Point;

public class MeshEditor 
{
    // these utils are designed for easy use from jython, but eventually would be nice in a control panel
    
    public static void addNode(FemModel3d fem, double x, double y, double z)
    {
        FemNode3d node = new FemNode3d(x,y,z);
        //node.setPosition(x, y, z);
        fem.addNode(node);
    }
    
    public static boolean addElement(FemModel3d fem, int[] indices)
    {
        FemNode3d[] nodes = new FemNode3d[indices.length];
        for (int i=0; i<indices.length; i++)
            nodes[i] = fem.getNodes().getByNumber(indices[i]);
        return addElement(fem, nodes);
    }
    
    public static boolean addElement(FemModel3d fem, FemNode3d[] nodes)
    {
        int nNodes = nodes.length;
        FemElement3d elem = null;
        if      (nNodes == 4)
            elem = new TetElement(nodes);
        else if (nNodes == 5)
            elem = new PyramidElement(nodes);
        else if (nNodes == 6)
            elem = new WedgeElement(nodes);
        else if (nNodes == 8)
            elem = new HexElement(nodes);
        else
            return false;
        
        if (elem.computeVolumes() < 0.0)
        {
            System.out.println("Element is inverted. Not added.");
            return false;
        }
        else
        {
            fem.addElement(elem);
            return true;
        }
        
    }
    
    public static PyramidElement[] decomposeHexTo6Pyramids(FemModel3d fem, FemElement3d elem)
    {
        // TODO: I add the new node but not the elements...kuldgy
        
        Point3d centroid = new Point3d();
        elem.computeCentroid(centroid);
        FemNode3d centNode = new FemNode3d(centroid);
        fem.addNode(centNode);
        
        FemNode3d[] allNodes = elem.getNodes();
        PyramidElement[] pe = new PyramidElement[6];
        
        pe[0] = new PyramidElement(allNodes[3], allNodes[2], allNodes[1], allNodes[0], centNode);
        pe[1] = new PyramidElement(allNodes[7], allNodes[6], allNodes[2], allNodes[3], centNode);
        pe[2] = new PyramidElement(allNodes[4], allNodes[5], allNodes[6], allNodes[7], centNode);
        pe[3] = new PyramidElement(allNodes[0], allNodes[1], allNodes[5], allNodes[4], centNode);
        pe[4] = new PyramidElement(allNodes[0], allNodes[4], allNodes[7], allNodes[3], centNode);
        pe[5] = new PyramidElement(allNodes[1], allNodes[2], allNodes[6], allNodes[5], centNode);
        
        return pe;
    }
    
    public static TetElement[] decomposePyramidTo4Tets(FemModel3d fem, FemElement3d elem)
    {

        FemNode3d[] pyNodes = elem.getNodes(); 
        Point3d basePoint = new Point3d();
        for (int a=0; a<4; a++)
            basePoint.add(pyNodes[a].getPosition());
        basePoint.scale(0.25);
        FemNode3d baseNode = new FemNode3d(basePoint);
        fem.addNode(baseNode);

        TetElement[] te = new TetElement[4];
        te[0] = new TetElement(pyNodes[0], pyNodes[1], baseNode, pyNodes[4]);
        te[1] = new TetElement(pyNodes[1], pyNodes[2], baseNode, pyNodes[4]);
        te[2] = new TetElement(pyNodes[2], pyNodes[3], baseNode, pyNodes[4]);
        te[3] = new TetElement(pyNodes[3], pyNodes[0], baseNode, pyNodes[4]);
        return te;
    }
    
    
    public static boolean elementContainsNode(FemElement3d elem, FemNode3d node)
    {
        for (FemNode3d elemNode : elem.getNodes())
            if (elemNode == node)
                return true;
        return false;
    }
        
    
    
    public static ArrayList<ArrayList<FemNode3d>> findDuplicateNodes(FemModel3d fem, double tol)
    {
        ArrayList<ArrayList<FemNode3d>> nodePairs = new ArrayList<ArrayList<FemNode3d>>(); 
        ArrayList<FemNode3d> dups = new ArrayList<FemNode3d>();
        int nNodes = fem.getNodes().size();
        
        for (int a=0; a<nNodes; a++)
        {
            FemNode3d node = fem.getNode(a);
            if (dups.contains(node) == true)
                continue;
            ArrayList<FemNode3d> pairings = new ArrayList<FemNode3d>();
            pairings.add(node);
            for (int b=a+1; b<nNodes; b++)
            {
                if ( node.getPosition().distance(fem.getNode(b).getPosition()) <= tol )
                {
                    pairings.add(fem.getNode(b));
                }
            }
            if (pairings.size() > 1)
            {
                for (FemNode3d n : pairings)
                    if (dups.contains(n) == false)
                        dups.add(n);
                nodePairs.add(pairings);
            }
        }
        return nodePairs;
    }
    
    
    public static void mergeDuplicateNodes(FemModel3d fem, double tol)
    {
        ArrayList<ArrayList<FemNode3d>> pairings = findDuplicateNodes(fem, tol);
        
        HashMap<FemNode3d, FemNode3d> nodeSwap = new HashMap<FemNode3d, FemNode3d>(pairings.size());
        for (ArrayList<FemNode3d> pairing : pairings)
        {
            Point3d mergePoint = new Point3d();
            for (FemNode3d node : pairing)
                mergePoint.add(node.getPosition());
            mergePoint.scale(1.0/((double) pairing.size()) );
            FemNode3d mergeNode = new FemNode3d(mergePoint);
            fem.addNode(mergeNode);
            for (FemNode3d node : pairing)
                nodeSwap.put(node, mergeNode);
        }
        
        for ( FemNode3d cutNode : nodeSwap.keySet() )
        {
            for (FemElement3d elem : cutNode.getElementDependencies())
            {
                for (int a=0; a<elem.numNodes(); a++)
                {
                    if (elem.getNodes()[a] == cutNode)
                    {
                        //cutNode.removeElementDependency(elem);
                        elem.getNodes()[a] = nodeSwap.get(cutNode);
                        nodeSwap.get(cutNode).addElementDependency(elem);
                    }
                }
                elem.connectToHierarchy(elem.getParent());
            }
            cutNode.getElementDependencies().clear();
            fem.removeNode(cutNode);
        }
        //fem.connectToHierarchy();
        
    }
    
    public static void reportDuplicateNodes(FemModel3d fem, double tol)
    {
        ArrayList<ArrayList<FemNode3d>> pairings = findDuplicateNodes(fem, tol);
        System.out.printf("%s contains %d duplicate nodes.\n", fem.getName(), pairings.size());
        for (ArrayList<FemNode3d> pairing : pairings)
        {
            System.out.printf("Set of %d duplicates: ", pairing.size());
            for (FemNode3d node : pairing)
            {
                System.out.printf("%d, ", node.myNumber);
            }
            System.out.println();
        }
    }
    
    public static boolean snapNodeToSurface(FemModel3d fem, int nodeNum, PolygonalMesh surface, double tol)
    {
        FemNode3d node = fem.getNodes().getByNumber(nodeNum);
        return snapPointToSurface(node, surface, tol);
    }
    
    public static ArrayList<Point> snapPointsToSurface(ArrayList<? extends Point> points, PolygonalMesh surface, double tol)
    {
        ArrayList<Point> snapped = new ArrayList<Point>();
        for (Point p : points)
            if (snapPointToSurface(p, surface, tol) == true)
                snapped.add(p);
        return snapped;
    }
    
    public static ArrayList<Point> snapPointsToSurface(ArrayList<? extends Point> points, PolygonalMesh surface)
    {
        ArrayList<Point> snapped = new ArrayList<Point>();
        for (Point p : points)
        {
            snapPointToSurface(p, surface);
            snapped.add(p);
        }
        return snapped;
    }
    
    public static boolean snapPointToSurface(Point p, PolygonalMesh surface, double tol)
    {
        Point3d nearPnt = nearestPointOnSurface(p.getPosition(), surface);
        if (p.getPosition().distance (nearPnt) > tol)
            return false;
        
        p.setPosition(nearPnt);
        return true;
    }
    
    public static boolean snapPointToSurface(Point p, PolygonalMesh surface)
    {
        Point3d nearPnt = nearestPointOnSurface(p.getPosition(), surface);
        p.setPosition(nearPnt);
        return true;
    }
    
    public static boolean snapVertexToSurface(Vertex3d vert, PolygonalMesh surface, double tol)
    {
        Point3d nearPnt = nearestPointOnSurface(vert.getPosition(), surface);
        if (vert.getPosition().distance (nearPnt) > tol)
            return false;
        
        vert.setPosition(nearPnt);
        return true;
    }
    
    public static void snapSurfaceToSurface(PolygonalMesh surface, PolygonalMesh surf_target, double tol)
    {
        for (Vertex3d vert: surface.getVertices())
            snapVertexToSurface(vert, surf_target, tol);
    }
    
    public static void snapInternalNodesToSurface(FemModel3d fem, PolygonalMesh surface)
    {
        for (FemNode3d node : fem.getNodes())
        {
            BVFeatureQuery query = new BVFeatureQuery();
            InsideQuery iq = query.isInsideMesh(surface, node.getPosition(), 0.0);
            if (iq == InsideQuery.INSIDE)
            {
                Point3d nearPnt = nearestPointOnSurface(node.getPosition(), surface);
                node.setPosition(nearPnt);
            }
        }
    }
    
    public static ArrayList<FemNode3d> findSurfaceNodes(FemModel3d fem)
    {
        ArrayList<FemNode3d> surfNodes = new ArrayList<FemNode3d>();
        for (FemNode3d node : fem.getNodes())
        {
            if (fem.isSurfaceNode(node) == true)
                surfNodes.add(node);
        }
        return surfNodes;
    }
    
    public static Point3d nearestPointOnSurface(Point3d point, PolygonalMesh surface)
    {
        BVFeatureQuery query = new BVFeatureQuery();
        Point3d nearPnt = new Point3d();
        Face nearFace = query.nearestFaceToPoint (nearPnt, null, surface, point);
        return nearPnt;
    }
    
    public static boolean makeSymmetrical(FemModel3d fem_sym, FemModel3d fem_orig, int axis, double axisValue, boolean isPositive)
    {
        // the general solution would use a cutplane and normal to define the "keeper" side while the opposite side is discarded
        // of course, such a case could always be transformed to a simple x,y,z symmetry and then use this approach...
        
        // make a fresh FEM model rather than modifying the old...
        // make a list of all throw-away nodes and elements
        // make a list of all keeper nodes
        // make a list of all symmetry nodes (that lie on symmetry plane)
        // an element is a keeper if it contains *only* keeper or symmetry nodes
        // an element is a throw away if it contains *only* throw away or symmetry nodes
        // an element is a symmetry element if it contains both keeper and throw-away nodes
        // make a paired list of all mirrored nodes
        // for each keeper element, make a mirror using the paired mirrored nodes
        // for each symmetry element, form it intelligently using the element with nSymNodes + 2*nKeepNodes ... if not possible --> error!
        
        double tol = 1e-9;  // tol for judging if a node lies on the symmetry plane
        double pm = 1.0;    // multiplier to switch signs if the negative side is the keeper
        if (isPositive == false)
            pm = -1.0;
        
        if (fem_sym == null)
            fem_sym = new FemModel3d("SymmetricFem");
        else
            fem_sym.clear();
        
        ArrayList<FemNode3d> keepNodes_old = new ArrayList<FemNode3d>();
        ArrayList<FemNode3d> tossNodes_old = new ArrayList<FemNode3d>();
        ArrayList<FemNode3d> symmNodes_old = new ArrayList<FemNode3d>();
        ArrayList<FemNode3d> keepNodes_new = new ArrayList<FemNode3d>();
        ArrayList<FemNode3d> mirrNodes_new = new ArrayList<FemNode3d>();
        ArrayList<FemNode3d> symmNodes_new = new ArrayList<FemNode3d>();
        
        // collect and mirror the FemNodes
        for ( FemNode3d node : fem_orig.getNodes() )
        {
            Point3d pos = node.getPosition();
            if      (Math.abs(pos.get(axis) - axisValue) < tol) // is pnt on symm plane, within tolerance
            {
                FemNode3d symmNode = new FemNode3d(pos.x, pos.y, pos.z);
                
                fem_sym.addNode(symmNode);
                symmNodes_old.add(node);
                symmNodes_new.add(symmNode);
                //renderNode(symmNode, 0.02, Color.blue); // TODO: temp rendering
            }
            else if ( (pos.get(axis) - axisValue)*pm > 0.0)
            {
                FemNode3d keepNode = new FemNode3d(pos.x, pos.y, pos.z);
                FemNode3d mirrNode = new FemNode3d(pos.x, pos.y, pos.z);
                mirrNode.getPosition().set(axis, 2.0*axisValue - pos.get(axis) );
                
                fem_sym.addNode(keepNode);
                fem_sym.addNode(mirrNode);
                keepNodes_old.add(node);
                keepNodes_new.add(keepNode);
                mirrNodes_new.add(mirrNode);
                //renderNode(keepNode, 0.02, Color.green); // TODO: temp rendering
                //renderNode(mirrNode, 0.02, Color.red); // TODO: temp rendering
            }
            
            else
            {
                tossNodes_old.add(node);
            }
            
        }
        
        // collect and mirror the FemElements
        for (FemElement3d elem : fem_orig.getElements())
        {
//            boolean hasKeepNodes = false;
//            boolean hasTossNodes = false;
            int nKeepNodes = 0;
            int nSymmNodes = 0;
            int nTossNodes = 0;
            
            FemNode3d[] origNodes = elem.getNodes();
            int nNodes = origNodes.length;
//            FemNode3d[] newNodes    = null; 
//            FemNode3d[] newNodesSym = null;
            
            for (FemNode3d node : origNodes)
            {
                if      (keepNodes_old.contains(node) == true)
                    nKeepNodes++;
                else if (symmNodes_old.contains(node) == true)
                    nSymmNodes++;
                else
                    nTossNodes++;
            }
            
            if      ( (nKeepNodes > 0) && (nTossNodes == 0) )
            {
                // this is a keep element: re-form this element using the 
                FemNode3d[] newNodes    = new FemNode3d[origNodes.length];
                FemNode3d[] newNodesSym = new FemNode3d[origNodes.length];
                for (int a=0; a<origNodes.length; a++)
                {
                    int ind = keepNodes_old.indexOf(origNodes[a]);
                    if (ind == -1)
                    {
                        ind = symmNodes_old.indexOf(origNodes[a]);
                        newNodes[a]    = symmNodes_new.get(ind);
                        newNodesSym[a] = symmNodes_new.get(ind);
                    }
                    else
                    {
                        newNodes[a]    = keepNodes_new.get(ind);
                        newNodesSym[a] = mirrNodes_new.get(ind);
                    }
                }
                // correct the orientation of the symmetrical element
                FemNode3d[] tempNodes = new FemNode3d[origNodes.length];
                if      (origNodes.length == 8)
                {
                    tempNodes[0] = newNodesSym[0];
                    tempNodes[3] = newNodesSym[1];
                    tempNodes[2] = newNodesSym[2];
                    tempNodes[1] = newNodesSym[3];
                    tempNodes[4] = newNodesSym[4];
                    tempNodes[7] = newNodesSym[5];
                    tempNodes[6] = newNodesSym[6];
                    tempNodes[5] = newNodesSym[7];
                    newNodesSym = tempNodes;
                }
                else if (origNodes.length == 6)
                {
                    tempNodes[0] = newNodesSym[0];
                    tempNodes[2] = newNodesSym[1];
                    tempNodes[1] = newNodesSym[2];
                    tempNodes[3] = newNodesSym[3];
                    tempNodes[5] = newNodesSym[4];
                    tempNodes[4] = newNodesSym[5];
                    newNodesSym = tempNodes;
                }
                else if (origNodes.length == 5)
                {
                    tempNodes[0] = newNodesSym[0];
                    tempNodes[3] = newNodesSym[1];
                    tempNodes[2] = newNodesSym[2];
                    tempNodes[1] = newNodesSym[3];
                    tempNodes[4] = newNodesSym[4];
                    newNodesSym = tempNodes;
                }
                else if (origNodes.length == 4)
                {
                    tempNodes[0] = newNodesSym[0];
                    tempNodes[2] = newNodesSym[1];
                    tempNodes[1] = newNodesSym[2];
                    tempNodes[3] = newNodesSym[3];
                    newNodesSym = tempNodes;
                }
                addElement(fem_sym, newNodes);
                addElement(fem_sym, newNodesSym);
            }
            else if ( nKeepNodes == nTossNodes )
            {
                // this is a symmetry element which is assumed to be cut in a topologically symmetrical way. 
//                if (nKeepNodes != nTossNodes)
//                    return false;
                FemNode3d[] newNodes = new FemNode3d[origNodes.length];
                for (int a=0; a<origNodes.length; a++)
                {
                    int ind = symmNodes_old.indexOf(origNodes[a]);
                    if      (symmNodes_old.contains(origNodes[a]) == true)
                        newNodes[a] = symmNodes_new.get( symmNodes_old.indexOf(origNodes[a]) );
                    else if (keepNodes_old.contains(origNodes[a]) == true)
                        newNodes[a] = keepNodes_new.get( keepNodes_old.indexOf(origNodes[a]) );
                    else
                    {
                        // this is a toss node, so find the neighbor that is a keep node, and use its mirror
                        FemNode3d tossNode = origNodes[a];
                        for (FemNode3d nn : getNodeNeighborsInElem(elem, tossNode))
                        {
                            if (symmNodes_old.contains(nn) == true)
                            {
                                newNodes[a] = mirrNodes_new.get( keepNodes_old.indexOf(origNodes[a]) );
                            }
                        }
                    }
                    
                }
            }
            else
            {
                // this is a toss-element --> just ignore it
            }
        }
        
        
        return true;
        
    }
    
    static ArrayList<FemNode3d> getNodeNeighborsInElem(FemElement3d elem, FemNode3d node)
    {
        ArrayList<FemNode3d> nn = new ArrayList<FemNode3d>();
        for (FemNode3d elemNode : elem.getNodes())
            for (FemNodeNeighbor nodeN : node.getNodeNeighbors())
                if (nodeN.getNode().equals(elemNode) == true)
                    nn.add(elemNode);
        return nn;
    }
    
    static void renderNode(FemNode3d node, double rad, Color color)
    {
        RenderProps rp = new RenderProps();
        rp.setPointColor(color);
        rp.setPointRadius(rad);
        rp.setPointStyle(PointStyle.SPHERE);
        node.setRenderProps(rp);
    }
    
    public static ArrayList<TetElement> selectTetElements(Iterable<? extends FemElement3d> elements)
    {
        ArrayList<TetElement> elems = new ArrayList<TetElement>();
        for (FemElement3d elem : elements)
            if ( elem instanceof TetElement )
                elems.add((TetElement)elem);
        return elems;
    }
    
    public static ArrayList<HexElement> selectHexElements(Iterable<? extends FemElement3d> elements)
    {
        ArrayList<HexElement> elems = new ArrayList<HexElement>();
        for (FemElement3d elem : elements)
            if ( elem instanceof HexElement )
                elems.add((HexElement)elem);
        return elems;
    }
    
    public static ArrayList<PyramidElement> selectPyramidElements(Iterable<? extends FemElement3d> elements)
    {
        ArrayList<PyramidElement> elems = new ArrayList<PyramidElement>();
        for (FemElement3d elem : elements)
            if ( elem instanceof PyramidElement )
                elems.add((PyramidElement)elem);
        return elems;
    }
    
    public static ArrayList<WedgeElement> selectWedgeElements(Iterable<? extends FemElement3d> elements)
    {
        ArrayList<WedgeElement> elems = new ArrayList<WedgeElement>();
        for (FemElement3d elem : elements)
            if ( elem instanceof WedgeElement )
                elems.add((WedgeElement)elem);
        return elems;
    }
    
    public static ArrayList<FemElement3d> selectElementsByVolume(FemModel3d fem, double minVol, double maxVol)
    {
        fem.updateVolume();
        ArrayList<FemElement3d> elems = new ArrayList<FemElement3d>();
        for (FemElement3d elem : fem.getElements())
            if ( (elem.getVolume() >= minVol) && (elem.getVolume() <= maxVol) )
                elems.add(elem);
        return elems;
    }
    
    public static ArrayList<FemNode3d> selectNodesByElemDep(FemModel3d fem, int minNum, int maxNum)
    {
        ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d>();
        for (FemNode3d node : fem.getNodes())
        {
            int nElems = node.getElementDependencies().size();
            if ( (nElems >= minNum) && (nElems <= maxNum) )
                nodes.add(node);
        }
        return nodes;
    }
    
    // Rendering Utils
    
    public static void highlightElementsByVolume(FemModel3d fem, double minVol, double maxVol, Color color)
    {
        fem.updateVolume();
        for (FemElement3d elem : fem.getElements())
        {
            if ( (elem.getVolume() >= minVol) && (elem.getVolume() <= maxVol) )
                elem.getRenderProps().setFaceColor(color);
            else
                elem.getRenderProps().setFaceColor(Color.gray);
        }
    }
    
    public static void highlightElementsByVolume(FemModel3d fem, double minVol, double maxVol)
    {
        highlightElementsByVolume(fem, minVol, maxVol, Color.red);
    }
    
    public static void highlightNodesByElemDep(FemModel3d fem, int minNum, int maxNum, Color color)
    {
        for (FemNode3d node : fem.getNodes())
        {
            int nElems = node.getElementDependencies().size();
            if ( (nElems >= minNum) && (nElems <= maxNum) )
                node.getRenderProps().setPointColor(color);
            else
                node.getRenderProps().setPointColor(Color.gray);
        }
    }
    
    public static void highlightNodesByElemDep(FemModel3d fem, int minNum, int maxNum)
    {
        highlightNodesByElemDep(fem, minNum, maxNum, Color.red);
    }
    
    // highlight points on surface
    // global FEM/Surf stats
    // element/node stats
    
    
//    public static ControlPanel createMeshEditor(MechModel mechModel) 
//    {
//        ControlPanel panel = new ControlPanel ("Mesh Editor", "LiveUpdate");
//        
//        // dropdown box defining which model we are editing
//        // text box with filename; save button
//        
//        panel.addWidget (new JSeparator());
//        panel.addLabel("Nodes");
//        panel.addWidget (new JSeparator());
//        panel.addLabel("Elements");
//        
//        
//        //panel.setLocation (Main.getMainFrame().getSize().width, 0);
//        panel.pack();
//        panel.setVisible(true);
//        
//        return panel;
//    }
    
    
//    public static void addSaveDialog (ControlPanel controlPanel) 
//    {
//        final Main main = Main.getMain();
////        JFrame frame = main.getMainFrame();
////        
////        final JDialog saveOptions = new JDialog (frame, "Save As VTK File");
//        
//        LabeledComponentPanel savePanel = new LabeledComponentPanel();
//        savePanel.setBorder (BorderFactory.createLineBorder(Color.blue));
//        
//        final StringField dirField = new StringField ("Output folder", main.getModelDirectory().getAbsolutePath(), 20);
//        dirField.setStretchable (true);
//        dirField.getTextField().setEditable(false);
//        
//        final JButton browseButton = new JButton ("Change");
//        browseButton.addActionListener (new ActionListener () {
//           public void actionPerformed (ActionEvent a_evt) 
//           {
//              JFileChooser chooser = new JFileChooser();
//              chooser.setCurrentDirectory (main.getModelDirectory ());
//              chooser.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
//              
//              if ( chooser.showSaveDialog (browseButton) == JFileChooser.APPROVE_OPTION) 
//              {
//                 dirField.setValue (chooser.getSelectedFile ().getAbsolutePath ());
//              }
//           }
//        });
//        browseButton.setMargin (new Insets (3, 3, 3, 3));
//        GuiUtils.setFixedSize (browseButton, new Dimension (80, 25));
//        dirField.add (browseButton);
//        savePanel.addWidget (dirField);
//        
//        final StringField nameField = new StringField ("File name", "mesh.vtk", 10);
//        nameField.setStretchable (true);
//        savePanel.addWidget (nameField);
//        
//        
//        OptionPanel dialogOptions = 
//           new OptionPanel ("Save Cancel", new ActionListener () {
//              public void actionPerformed (ActionEvent a_evt) {
//                 if (a_evt.getActionCommand ().equals ("Save")) {
//                    File directory = new File (dirField.getStringValue ());
//                    File file = new File (directory, nameField.getStringValue ());
//                    
//                    VtkInputOutput.writeVTK (file.getAbsolutePath(), model);
//                 } 
//                 
//                 //saveOptions.dispose ();
//              }
//           });
//        
//        GuiUtils.setFixedSize (
//           dialogOptions.getButton ("Save"), new Dimension (85, 25));
//        GuiUtils.setFixedSize (
//           dialogOptions.getButton ("Cancel"), new Dimension (85, 25));
//        dialogOptions.setBorder (BorderFactory.createEmptyBorder (0, 0, 8, 0));
//        savePanel.addWidget (dialogOptions);
//        
//        controlPanel.addWidget(savePanel);
////        saveOptions.getContentPane ().add (savePanel);      
////        saveOptions.pack ();
////        saveOptions.setMinimumSize (saveOptions.getPreferredSize ());
////        saveOptions.setVisible (true);
//        
//     }

}
