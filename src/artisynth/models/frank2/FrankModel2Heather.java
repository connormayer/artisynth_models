package artisynth.models.frank2;


import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JSeparator;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.femmodels.PointFem3dAttachment;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.femmodels.VtkInputOutput;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.FemControlPanel;
import artisynth.core.inverse.TrackingController;
import artisynth.core.inverse.InverseManager;
import artisynth.core.materials.AxialMaterial;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.MooneyRivlinMaterial;
import artisynth.core.materials.IncompressibleMaterialBase.BulkPotential;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameAttachment;
import artisynth.core.mechmodels.Marker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.fluid1d.CenterlineLinear;
import artisynth.models.frank2.FrankActivations;
import artisynth.models.frank2.FrankAttachments;
import artisynth.models.frank2.FrankModel2;
import artisynth.models.frank2.FrankMuscles;
import artisynth.models.frank2.GenericModel;
import artisynth.models.frank2.FrankMuscles.MuscleSymmetry;
import artisynth.models.frank2.frankUtilities.MeshEditor;
import artisynth.models.frank2.frankUtilities.StopWatch;
import artisynth.models.frank2.frankUtilities.StopWatch.Units;
import maspack.collision.IntersectionContour;
import maspack.collision.SurfaceMeshContourIxer;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.PlyWriter;
import maspack.geometry.io.WavefrontReader;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.GL.GLViewer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;
import maspack.widgets.GuiUtils;


public class FrankModel2Heather extends FrankModel2 {
   

   /*/ TODO:
    * Model organization:
    *  - remove face, larynx, spine? Unnecessary components
    *  - change all attachments to alter according to imported geometry (create attachment surfaces to replace all dependencies on meshes)
    *  - import new meshes 
    *  - "ligaments" grouping is incomplete --> many larynx "muscles" are ligaments...
    *  
    *  
    * Model functionality:
    *  
    *  - larynx collisions cause errors
    *  - appropriate muscle forces for the pharynx
    *  - 
    *  - make external muscles symmetrical (should be nearly so)
    *  - add support for a symmetrical simulation
    *  
    * Misc:
    *  - 
    *  - control panel: add switches to de-activate models as needed!
    *  - 
    *  - have no dependencies outside of artisynth_core and maspack
    *  - 
    *  - organization: SoftTissue, Bone, Ligament, Muscles
   /*/


    public MechModel mechModel = new MechModel();
    
    public String modelDir   = ArtisynthPath.getSrcRelativePath (FrankModel2Heather.class, "");
    public String outputDir   = modelDir + "output/";
    public String geometryDir = modelDir + "geometry/";
    boolean tempOutputDir = true;
    
    protected RenderableComponentList<FemMuscleModel> fems = new RenderableComponentList<FemMuscleModel>(FemMuscleModel.class, "DeformableBodies");
    protected RenderableComponentList<RigidBody> rbs = new RenderableComponentList<RigidBody>(RigidBody.class, "RigidBodies");
    //RenderableComponentList<MuscleBundle> muscles = new RenderableComponentList<MuscleBundle>(MuscleBundle.class, "muscles");
    //RenderableComponentList<RenderableComponentList> muscles = new RenderableComponentList<RenderableComponentList>(RenderableComponentList.class, "muscles");
    protected ArrayList<RenderableComponentList> muscles = new ArrayList<RenderableComponentList>(); // use array list if not adding to mechModel
    protected RenderableComponentList<MuscleBundle> faceMuscles, tongueMuscles, palateMuscles, pharynxMuscles, larynxMuscles;
    protected RenderableComponentList<MuscleBundle> externalMuscles = new RenderableComponentList<MuscleBundle>(MuscleBundle.class, "ExternalMuscles");
    protected RenderableComponentList<MuscleBundle> ligaments       = new RenderableComponentList<MuscleBundle>(MuscleBundle.class, "Ligaments");
    protected ComponentList<MuscleExciter> faceExciters, tongueExciters, palateExciters, pharynxExciters, larynxExciters, externalExciters;
    protected ComponentList[] exciterGroups;// = {faceExciters, tongueExciters, palateExciters, pharynxExciters, larynxExciters, externalExciters};

    // Deformable components
    public FemMuscleModel tongue = new FemMuscleModel();
    public FemMuscleModel face = new FemMuscleModel();
    public FemMuscleModel softPalate = new FemMuscleModel();
    public FemMuscleModel larynx = new FemMuscleModel();
    public FemMuscleModel pharynx = new FemMuscleModel();
    // Rigid Components
    RigidBody ground = new RigidBody("ground");
    RigidBody hyoid = new RigidBody("hyoid");
    RigidBody maxilla  = new RigidBody("maxilla");
    RigidBody mandible  = new RigidBody("mandible");
    RigidBody jaw = new RigidBody("jaw");
    RigidBody thyroid = new RigidBody("thyroid");
    RigidBody cricoid = new RigidBody("cricoid");
    RigidBody epiglottis = new RigidBody("epiglottis");
    RigidBody cranium = new RigidBody("cranium");
    RigidBody cuneiform_L = new RigidBody("cuniform_L");
    RigidBody cuneiform_R = new RigidBody("cuniform_R");
    RigidBody arytenoid_L = new RigidBody("arytenoid_L");
    RigidBody arytenoid_R = new RigidBody("arytenoid_R");
    RigidBody skull = new RigidBody("skull");
    RigidBody palateRB = new RigidBody("palateRB");
    double rigidBodyFrameDamping = 5.0;
    double rigidBodyRotaryDamping = 100.0/(1000.0*1000.0);
    // Skin components
    SkinMeshBody airway = new SkinMeshBody();
    SkinMeshBody airway_tube = new SkinMeshBody();
    SkinMeshBody centerlineSkin;

//    public enum NodeCaching{Load, Ignore, Save};
//    NodeCaching nodeCaching = NodeCaching.Ignore;
    
    protected boolean excitersCombineLR = true;
    protected boolean useShortMuscleNames = true;
    
    protected boolean saveFrames = false; // save screenshots each time step to output directory
    protected boolean saveGeometries = false;

    public void build (String[] args) throws IOException 
    {
        StopWatch stopwatch = new StopWatch("Main Timer", Units.seconds);
        
        if (tempOutputDir == true)
        {
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy.MM.dd_HH.mm.ss");
            outputDir   = outputDir + "temp_" + dateFormat.format(new Date()) + "/";
            //outputDir   = outputDir + "temp_" + Long.toString(System.nanoTime()) + "/";
            GenericModel.createDirectory(outputDir);
        }
        
        addModel(mechModel);
        mechModel.setName("FrankMechModel");
        mechModel.setGravity(0.0, 0.0, -9.81);
        //mechModel.setGravity(0.0, 0.0, 0.0);
        mechModel.add(fems);
        mechModel.add(rbs);
        mechModel.add(externalMuscles);
        mechModel.add(ligaments);
        
        face.getMuscleBundles().setName("FaceMuscles");
        tongue.getMuscleBundles().setName("TongueMuscles");
        softPalate.getMuscleBundles().setName("PalateMuscles");
        pharynx.getMuscleBundles().setName("PharynxMuscles");
        larynx.getMuscleBundles().setName("LarynxMuscles");
     //   faceMuscles = face.getMuscleBundles();
        tongueMuscles = tongue.getMuscleBundles();
        palateMuscles = softPalate.getMuscleBundles();
        pharynxMuscles = pharynx.getMuscleBundles();
        larynxMuscles = larynx.getMuscleBundles();
        
        //mechModel.add(muscles);
        muscles.add(faceMuscles);
        muscles.add(tongueMuscles);
        muscles.add(palateMuscles);
        muscles.add(pharynxMuscles);
        muscles.add(larynxMuscles);
        muscles.add(externalMuscles);
        muscles.add(ligaments);
        
        this.setMaxStepSize(0.005);  // don't use mechModel.setMaxStepSize (0.005);
        this.setAdaptiveStepping(false);

        ArtisynthPath.setWorkingDir( ArtisynthPath.getSrcRelativeFile (this, "") );
        
        stopwatch.checkpoint("Basic initializations done.");

        // --- loading in rigid bodies --- //
        
        // ground is an imaginary, fixed object to which some objects attach.
        ground.setMesh(MeshFactory.createSphere(0.001, 10, 0.0, 0.0, 0.0));
        ground.setDynamic(false);
        ground.getRenderProps().setVisible(false);
        rbs.add(ground);
        //"mandible_source_2_onJawVtk.ply"
        loadRigidBody(jaw,          "jaw",          geometryDir, "jaw.ply",         3470.0);
        loadRigidBody(maxilla,      "maxilla",      geometryDir, "maxilla.ply",     1900.0);
        maxilla.setDynamic(false);
        loadRigidBody(mandible,      "mandible",      geometryDir, "mandible.ply",     1900.0);
        mandible.setDynamic(false);
        loadRigidBody(hyoid,        "hyoid",        geometryDir, "hyoid.ply",       1900.0);
        loadRigidBody(thyroid,      "thyroid",      geometryDir, "thyroid.ply",     1900.0);
        loadRigidBody(cricoid,      "cricoid",      geometryDir, "cricoid.ply",     1900.0);
        loadRigidBody(epiglottis,   "epiglottis",   geometryDir, "epiglottis.ply",  1900.0);
        loadRigidBody(cranium,      "cranium",      geometryDir, "cranium.ply",     1900.0);
        cranium.setDynamic(false);
        cranium.getRenderProps().setVisible(false);
  //      loadRigidBody(cuneiform_L,  "cuneiform_L",  geometryDir, "cuneiform_L.vtk", 1900.0);
  //      loadRigidBody(cuneiform_R,  "cuneiform_R",  geometryDir, "cuneiform_R.vtk", 1900.0);
        loadRigidBody(arytenoid_L,  "arytenoid_L",  geometryDir, "arytenoid_L.ply", 1900.0);
        loadRigidBody(arytenoid_R,  "arytenoid_R",  geometryDir, "arytenoid_R.ply", 1900.0);
//        loadRigidBody(skull,        "skull",        geometryDir, "badinskull.obj", 1900.0);
//        skull.setDynamic(false);
        loadRigidBody(palateRB, "palateRB", geometryDir, "palateRB.ply", 1900.0);
        
        stopwatch.checkpoint("Rigid bodies loaded.");
        
        
        // --- Loading in FEMs (deformable meshes) --- //
        
        String tongueMeshName = "tongue";
        GenericModel.loadFemMesh_VTK (tongue, geometryDir, tongueMeshName);
        fems.add(tongue);
        tongue.setName ("tongue");
        MooneyRivlinMaterial tongueMaterial = new MooneyRivlinMaterial();
        tongueMaterial.setBulkModulus(10370.0);
        tongueMaterial.setBulkPotential(BulkPotential.QUADRATIC);
        tongueMaterial.setC01(1037.0);
        tongueMaterial.setC10(0.0);
        tongueMaterial.setC11(0.0);
        tongueMaterial.setC20(486.0);
        tongueMaterial.setC02(0.0);
        tongueMaterial.setJLimit(0.0);
        tongue.setMaterial (tongueMaterial);
        tongue.setDensity (1040.0);
        tongue.setIncompressible (IncompMethod.AUTO);
        tongue.setParticleDamping(10.0);
        tongue.setStiffnessDamping(0.03);
        
        stopwatch.checkpoint("Loading FEMs: tongue loaded");


        //*/
        String palateMeshName = "palate"; // palate, palate_purehex, softPalate, palate_hex, palate_new_hex, palate_coarse, palate_fine01
        GenericModel.loadFemMesh(softPalate, geometryDir, palateMeshName);
        PolygonalMesh palateSurf = softPalate.getSurfaceMesh();
        /*/
        String palateMeshName = "palate_purehex"; // palate, palate_purehex, softPalate, palate_hex, palate_new_hex, palate_coarse, palate_fine01
        GenericModel.loadFemMesh(softPalate, geometryDir, palateMeshName);
        PolygonalMesh palateSurf = GenericModel.loadGeometry(geometryDir, "palate_coarse_surf.ply");
        softPalate.addMesh(palateSurf);
        //*/
        fems.add(softPalate);
        softPalate.setName ("softPalate");
        
        //*/
        LinearMaterial palateMaterial = new LinearMaterial();
        palateMaterial.setPoissonsRatio (0.4995); //0.4995
        palateMaterial.setYoungsModulus (500.0);
        /*/
        double palate_E = 500.0;
        double palate_nu = 0.45;
        double K = palate_E/(3.0*(1.0-2.0*palate_nu));    // bulk modulus  --> from wiki
        double G = palate_E/(2.0*(1.0+palate_nu));    // shear modulus --> from wiki
        IncompNeoHookeanMaterial palateMaterial = new IncompNeoHookeanMaterial();
        palateMaterial.setBulkModulus(K);
        palateMaterial.setShearModulus(G);
//        MooneyRivlinMaterial palateMaterial = new MooneyRivlinMaterial();    // c10=1037, c20=486, bm=10*c10
//        palateMaterial.setBulkModulus (K);
//        palateMaterial.setC01(G/2.0 - K/10.0);
//        palateMaterial.setC10 (K/10.0);                                   // Pa  [Buchaillard 2009],[Duck 1990]
        //*/
        
        //softPalate.setIncompCompliance(c);
        softPalate.setMaterial (palateMaterial);
        softPalate.setIncompressible (IncompMethod.OFF);
        softPalate.setDensity (1040.0); // 1040.0
        softPalate.setParticleDamping(0.0);
        softPalate.setStiffnessDamping(0.03);
        
        stopwatch.checkpoint("Loading FEMs: palate loaded");

        
        GenericModel.loadFemMesh_VTK (pharynx, geometryDir, "pharynx");
        fems.add(pharynx);
        pharynx.setName ("pharynx");
        LinearMaterial pharynxMaterial = new LinearMaterial();
        pharynxMaterial.setPoissonsRatio (0.49);
        pharynxMaterial.setYoungsModulus (15000.0); // Xu 2009?, Kim 2006 reports very large values
        pharynx.setMaterial (pharynxMaterial);
        pharynx.setIncompressible (IncompMethod.OFF);
        pharynx.setDensity (1040.0);
        pharynx.setParticleDamping(10.0);
        pharynx.setStiffnessDamping(0.03);
        
        stopwatch.checkpoint("Loading FEMs: pharynx loaded");


        

        String larynxMeshName = "larynx";
        GenericModel.loadFemMesh_VTK (larynx, geometryDir, larynxMeshName);
        fems.add(larynx);
        larynx.setName ("larynx");
        //      LinearMaterial larynxMaterial = new LinearMaterial();
        //      larynxMaterial.setPoissonsRatio (0.49);
        //      larynxMaterial.setYoungsModulus (15000.0);
        MooneyRivlinMaterial larynxMaterial = new MooneyRivlinMaterial();
        larynxMaterial.setBulkModulus(25000.0);
        larynxMaterial.setBulkPotential(BulkPotential.QUADRATIC);
        larynxMaterial.setC01(2500.0);
        larynxMaterial.setC10(0.0);
        larynxMaterial.setC11(0.0);
        larynxMaterial.setC20(1175.0);
        larynxMaterial.setC02(0.0);
        larynxMaterial.setJLimit(0.0);
        larynx.setMaterial (larynxMaterial);
        larynx.setIncompressible (IncompMethod.OFF);
        larynx.setDensity (1040.0);
        larynx.setParticleDamping(10.0);
        larynx.setStiffnessDamping(0.03);
        
        stopwatch.checkpoint("Loading FEMs: larynx loaded");
        
        // --- attachments --- //
        
        // jaw, hyoid
        FrankAttachments.defineJawContraints(jaw, cranium, mechModel);
        //FrankAttachments.defineHyoidConstraints(hyoid, maxilla, mechModel);
        
        // tongue
        //attachFemToRigidBody(tongue, hyoid, 2.0/1000.0, true, nodeCaching, tongueMeshName);
        GenericModel.attachFemToRigidBody (tongue, hyoid, 2.0/1000.0, true, mechModel);
        GenericModel.attachFemToRigidBody(tongue, jaw, 
            GenericModel.loadGeometry(geometryDir, "attachmentSurf_tongue,jaw.ply"), 0.001, mechModel);
        //attachFemToRigidBody(tongue, jaw, tongueMeshName); // mesh dependent calculation
        
        // soft palate
        GenericModel.attachFemToRigidBody(softPalate, maxilla, 5.0/1000.0, true, mechModel);
        
        // pharynx
      //attach palate to pharynx 
        ArrayList<FemNode3d> nodesToAttach = GenericModel.findNodesNearSurface(pharynx, palateSurf, 0.0005, true);
        MeshEditor.snapPointsToSurface(nodesToAttach, softPalate.getSurfaceMesh());
        GenericModel.attachFemNodesToFem(pharynx, nodesToAttach, softPalate, mechModel);
//        ArrayList<FemNode3d> nodesToAttach = GenericModel.findNodesNearSurface(softPalate, pharynx.getSurfaceMesh(), 0.0005, true);
//        MeshEditor.snapPointsToSurface(nodesToAttach, pharynx.getSurfaceMesh());
//        GenericModel.attachFemNodesToFem(softPalate, nodesToAttach, pharynx, mechModel);
//        for (FemNode3d node : nodesToAttach)
//        {
//            LinkedList<FemElement3d> elems = node.getElementDependencies();
//            for (FemElement3d elem : elems)
//                elem.setMaterial(new LinearMaterial(10.0, 0.4));
//        }
        // ---
        //GenericModel.attachFemToFem(pharynx, softPalate, 0.0005, mechModel);
        //GenericModel.attachFemToFem(softPalate, pharynx, 0.0005, mechModel);
        GenericModel.attachFemToFem(pharynx, tongue, 
            GenericModel.loadGeometry(geometryDir, "attachmentSurf_pharynx,tongue.ply"), 0.0005, mechModel);
        GenericModel.attachFemToRigidBody(pharynx, thyroid, 0.001, true, mechModel);
        //GenericModel.attachFemToRigidBody(pharynx, cricoid, 0.001, true, mechModel); TODO: this would be an appropriate attachement to implement.
        //GenericModel.attachFemToRigidBody(pharynx, maxilla, 3.0/1000.0, true, mechModel);
        //for (FemNode3d node : pharynx.getNodes())
        for (FemNode3d node : GenericModel.findNodesNearSurface(pharynx, GenericModel.loadGeometry(geometryDir, "pharynx_posterior_surf.ply"), 0.0005, true))
        {
            if ( (node.getPosition().z > 0.128) || (node.getPosition().z < 0.017) || (Math.abs(node.getPosition().y) < 0.001) )
                node.setDynamic(false);
//            if ( (node.getPosition().z > 0.128) || (node.getPosition().z < 0.017) )
//                node.setDynamic(false);
//            if ( (node.getPosition().z > 0.121) && (Math.abs(node.getPosition().y) < 0.005) )
//                node.setDynamic(false);
        }
        
        
        // larynx
        GenericModel.attachFemToRigidBody(larynx, hyoid,       1.0/1000.0, true, mechModel);
        GenericModel.attachFemToRigidBody(larynx, epiglottis,  0.5/1000.0, false, mechModel); // false
        GenericModel.attachFemToRigidBody(larynx, thyroid,     1.0/1000.0, true, mechModel); // crasher
        GenericModel.attachFemToRigidBody(larynx, cricoid,     1.0/1000.0, true, mechModel); // crasher
        GenericModel.attachFemToRigidBody(larynx, cuneiform_L, 1.5/1000.0, true, mechModel); 
        GenericModel.attachFemToRigidBody(larynx, cuneiform_R, 1.5/1000.0, true, mechModel);
        GenericModel.attachFemToRigidBody(larynx, arytenoid_L, 1.0/1000.0, true, mechModel);
        GenericModel.attachFemToRigidBody(larynx, arytenoid_R, 1.0/1000.0, true, mechModel);
                
        // TODO: attach pharynx to hyoid
        
        stopwatch.checkpoint("Attachements defined.");

        // --- Adding Line Based muscles --- //
        addMuscles();
        stopwatch.checkpoint("All muscles added.");
        
        // --- Skin Meshes --- //
        //airway      = addAirwaySkin("airway_real", "airway_complete_jagged.ply");
        //airway_tube = addAirwaySkin("airway_tube", "airway_complete_tube.ply");
        addAirwaySkin(airway, "airway_real", "airway_complete_jagged.ply");
        addAirwaySkin(airway_tube, "airway_tube", "airway_complete_tube.ply");
        centerlineSkin = addCenterlineSkin("centerlineSkin", "centerlinePoints.csv");
        stopwatch.checkpoint("Skin meshes added.");

        // --- define the collision behaviors --- //
        mechModel.setCollisionBehavior(tongue, softPalate, true);
        mechModel.setCollisionBehavior(tongue, larynx, true);
        //mechModel.setCollisionBehavior(tongue, pharynx, true); //??
        mechModel.setCollisionBehavior(tongue, jaw, true);
        mechModel.setCollisionBehavior(tongue, maxilla, true);
        face.addMesh(GenericModel.loadGeometry_VTK(geometryDir + "face_collisionSurf_upperLip_reduced2.vtk"));
        face.addMesh(GenericModel.loadGeometry_VTK(geometryDir + "face_collisionSurf_lowerLip_reduced.vtk"));
        mechModel.setCollisionBehavior(face, face, true);
        mechModel.setCollisionBehavior(face, maxilla, true);
        mechModel.setCollisionBehavior(face, jaw, true);
        {
            PolygonalMesh palateColl = GenericModel.loadGeometry(geometryDir, "palate_collisionSurf_velum_fine.ply"); // _02
            palateColl.triangulate();
            //MeshEditor.snapSurfaceToSurface(palateColl, softPalate.getSurfaceMesh(), 0.01);
            MeshEditor.snapSurfaceToSurface(palateColl, palateSurf, 0.01);
            FemMeshComp palateUvula = softPalate.addMesh(palateColl);
            palateUvula.setName("VelumSurf");
            palateUvula.setCollidable(Collidability.EXTERNAL);
            mechModel.setCollisionBehavior(palateUvula, pharynx, true);
        }
        /*/ Add larynx self-collisions
        String[] surfNames = {"larynx_collisionSurf_AEFold_L.ply", "larynx_collisionSurf_AEFold_R.ply", "larynx_collisionSurf_FalseFold_L.ply", "larynx_collisionSurf_FalseFold_R.ply", "larynx_collisionSurf_VocalFold_L.ply", "larynx_collisionSurf_VocalFold_R.ply", "larynx_collisionSurf_epiglotticTubercle.ply"};
        for (String surfName : surfNames)
        {
            PolygonalMesh collSurface = GenericModel.loadGeometry(geometryDir, surfName);
            MeshEditor.snapSurfaceToSurface(collSurface, larynx.getSurfaceMesh(), 0.01);
            FemMesh collMesh = larynx.addMesh(collSurface);
            collMesh.setCollidable(Collidability.INTERNAL);
        }
        mechModel.setCollisionBehavior(larynx, larynx, true);
        //*/       
        
        //mechModel.getCollisionManager().setCollisionCompliance(1e-5);
        //mechModel.getCollisionManager().setCollisionDamping(100.0);
        //CollisionHandler.doBodyFaceContact = true;
        
        stopwatch.checkpoint("Collision behaviors defined.");


        // --- Rendering --- //
        defineRendering();
        stopwatch.checkpoint("Rendering defined.");
        
        // --- Control Panels --- //
        buildControlPanels();
        stopwatch.checkpoint("Control panels built.");
        // --- Control Panels --- //
        
        
        //createSynthEmmaPoints();
        
        /*/ Define some non-simulating fems
        FemModel3d[] nonDynamicFems = {face, larynx};
        for (FemModel3d fem : nonDynamicFems)
            for (FemNode3d node : fem.getNodes())
                node.setDynamic(false);
        //*/
        
        //writeAllMuscles("muscles/");

        //*/ some final checkups...
        for (FemModel3d fem : fems)
        {
            System.out.printf("Reports for %s:\n", fem.getName());
            System.out.printf("Stiffness Damping = %f, mass damping = %f, penetration limit = %f:\n", 
                fem.getStiffnessDamping(), fem.getParticleDamping(), fem.getPenetrationLimit());
            GenericModel.reportFemElements(fem);
            //GenericModel.findDisconnectedNodes(fem, true, false);
            System.out.println();
        }
        stopwatch.checkpoint("FEM Reports done");
        //*/
        
        GenericModel.writeClassState(this, outputDir + "classState.txt");
        stopwatch.checkpoint("Frank initializations done.");

        //System.out.println("Adding tracking controller"); // part of EMAInv model instead?
        //addTrackingController();
        
        //setUpperFaceStatic();
        //buildCutPlanes();
    }
    
   
    
    private void buildCutPlanes(){
       Point3d centroid = new Point3d();
       for(int i = 0; i < 5; i++){
          centroid.setZero ();
          centroid.scaledAdd (i/(double)8, face.getNodes ().get ("1052").getPosition ());
          centroid.scaledAdd (i/(double)8, face.getNodes ().get ("1053").getPosition ());
          centroid.scaledAdd ((4-i)/(double)8, face.getNodes ().get ("6384").getPosition ());
          centroid.scaledAdd ((4-i)/(double)8, face.getNodes ().get ("1996").getPosition ());
          RotationMatrix3d R = new RotationMatrix3d();
          R.setZDirection (new Vector3d(1,0,0));
          RigidTransform3d T = new RigidTransform3d(centroid,R.getAxisAngle ());
          PolygonalMesh mesh = MeshFactory.createRectangle (0.05, 0.05, true);
          mesh.setMeshToWorld (T);
          MeshComponent comp = new MeshComponent();
          comp.setMesh (mesh);
          mechModel.add (comp);
       }
    }
    
    public List<Double> getXSectionAreas (boolean recompute) {
       ArrayList<Double> toRet = new ArrayList<> ();
       List<Double> areas = new ArrayList<>();
       if (recompute) {
          computeXsectionalAreasAndGenerateContours (areas);
       }
       toRet.addAll (areas);
       return toRet;
    }
    
    protected List<Double> computeXsectionalAreasAndGenerateContours (List<Double> areas) {
       SurfaceMeshContourIxer intersector = new SurfaceMeshContourIxer();
       for(int i = 22 ; i < 27 ; i++){
          String comppath = "models/FrankMechModel/"+Integer.toString(i);
          MeshComponent comp = (MeshComponent)findComponent(comppath);
          boolean collided =
             intersector.findContours (
                (PolygonalMesh)comp.getMesh (),
                face.getSurfaceMesh ());
          if (collided) {
             areas.add (useSubtractSmallFromBigApproach (intersector));
          }
       }
       return areas;
    }
    
    protected double useSubtractSmallFromBigApproach (SurfaceMeshContourIxer intersector) {
       double finalArea = 0;
       double largestArea = 0;
       int largestAreaContourIdx = -1;
       for (int j = 0; j < intersector.getContours ().size (); j++) {
          IntersectionContour contour = intersector.getContours ().get (j);
          double area = contour.computePlanarArea ();
          if (area > largestArea && contour.isClosed()) {
             largestArea = area;
             largestAreaContourIdx = j;
          }
       }

       if (largestAreaContourIdx == -1) {
          finalArea = 0;
       }
       else {
          finalArea = largestArea; // Add largest area.
          for (int j = 0; j < intersector.getContours ().size (); j++) {
             IntersectionContour contour = intersector.getContours ().get (j);
             if (j != largestAreaContourIdx) {
                if (contour.isClosed ()) {
                   double area = contour.computePlanarArea ();
                   finalArea -= area; // Subtract "island" areas.
                }
             }
          }
       }
       return finalArea;
    }
    
    void addMuscles()
    {
        String faceMuscDir    = modelDir + "muscles/FaceMuscles/";
        String tongueMuscDir  = modelDir + "muscles/TongueMuscles/";
        String palateMuscDir  = modelDir + "muscles/PalateMuscles/";
        String pharynxMuscDir = modelDir + "muscles/PharynxMuscles/";
        String larynxMuscDir  = modelDir + "muscles/LarynxMuscles/";
        String externalMuscDir= modelDir + "muscles/ExternalMuscles/";
        String ligamentDir    = modelDir + "muscles/Ligaments/";
        
        //*/ --- external (jaw) muscles --- 
        loadMuscle("AT_L", "AnteriorTemporal_L",         null, FrankMuscles.peckMuscle(0.001, 158.0, 0.015, 0.974484, 1.237391, 0.5, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("AT_R", "AnteriorTemporal_R",         null, FrankMuscles.peckMuscle(0.001, 158.0, 0.015, 0.974242, 1.237083, 0.5, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("MT_L", "MiddleTemporal_L",           null, FrankMuscles.peckMuscle(0.001, 95.6, 0.015, 0.945072, 1.340707, 0.48, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("MT_R", "MiddleTemporal_R",           null, FrankMuscles.peckMuscle(0.001, 95.6, 0.015, 0.945341, 1.341089, 0.48, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("PT_L", "PosteriorTemporal_L",        null, FrankMuscles.peckMuscle(0.001, 75.6, 0.015, 0.933588, 1.223799, 0.51, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("PT_R", "PosteriorTemporal_R",        null, FrankMuscles.peckMuscle(0.001, 75.6, 0.015, 0.933585, 1.223794, 0.51, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("SM_L", "SuperficialMasseter_L",      null, FrankMuscles.peckMuscle(0.001, 190.4, 0.015, 0.958640, 1.245897, 0.46, 1.0),   false, null, externalMuscDir, externalMuscles);
        loadMuscle("SM_R", "SuperficialMasseter_R",      null, FrankMuscles.peckMuscle(0.001, 190.4, 0.015, 0.958353, 1.245524, 0.46, 1.0),   false, null, externalMuscDir, externalMuscles);
        loadMuscle("DM_L", "DeepMasseter_L",             null, FrankMuscles.peckMuscle(0.001, 81.6, 0.015, 0.969251, 1.495387, 0.29, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("DM_R", "DeepMasseter_R",             null, FrankMuscles.peckMuscle(0.001, 81.6, 0.015, 0.968306, 1.493929, 0.29, 1.0),    false, null, externalMuscDir, externalMuscles);
        loadMuscle("MP_L", "MedialPterygoid_L",          null, FrankMuscles.peckMuscle(0.001, 174.8, 0.015, 0.980787, 1.225802, 0.64, 1.0),   false, null, externalMuscDir, externalMuscles);
        loadMuscle("MP_R", "MedialPterygoid_R",          null, FrankMuscles.peckMuscle(0.001, 174.8, 0.015, 0.981646, 1.226876, 0.64, 1.0),   false, null, externalMuscDir, externalMuscles);
        loadMuscle("SP_L", "SuperiorLateralPterygoid_L", null, FrankMuscles.peckMuscle(0.001, 28.671429, 0.015, 1.111862, 1.513257, 0.0, 1.0),false, null, externalMuscDir, externalMuscles);
        loadMuscle("SP_R", "SuperiorLateralPterygoid_R", null, FrankMuscles.peckMuscle(0.001, 28.671429, 0.015, 1.113662, 1.515706, 0.0, 1.0),false, null, externalMuscDir, externalMuscles);
        loadMuscle("IP_L", "InferiorLateralPterygoid_L", null, FrankMuscles.peckMuscle(0.001, 66.9, 0.015, 1.048072, 1.380793, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("IP_R", "InferiorLateralPterygoid_R", null, FrankMuscles.peckMuscle(0.001, 66.9, 0.015, 1.051230, 1.384953, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("AD_L", "AnteriorDigastric_L",        null, FrankMuscles.peckMuscle(0.001, 40.0, 0.015, 1.176382, 1.511534, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("AD_R", "AnteriorDigastric_R",        null, FrankMuscles.peckMuscle(0.001, 40.0, 0.015, 1.175792, 1.510775, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("PD_L", "PosteriorDigastric_L",       null, FrankMuscles.peckMuscle(0.001, 40.0, 0.015, 1.0, 1.284900, 0.0, 1.0),          false, null, externalMuscDir, externalMuscles);
        loadMuscle("PD_R", "PosteriorDigastric_R",       null, FrankMuscles.peckMuscle(0.001, 40.0, 0.015, 1.0, 1.284900, 0.0, 1.0),          false, null, externalMuscDir, externalMuscles);
        loadMuscle("AM_L", "AnteriorMylohyoid_L",        null, FrankMuscles.peckMuscle(0.001, 35.4, 0.015, 1.225979, 1.575261, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("AM_R", "AnteriorMylohyoid_R",        null, FrankMuscles.peckMuscle(0.001, 35.4, 0.015, 1.227963, 1.577810, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("PM_L", "PosteriorMylohyoid_L",       null, FrankMuscles.peckMuscle(0.001, 35.4, 0.015, 1.095825, 1.408025, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("PM_R", "PosteriorMylohyoid_R",       null, FrankMuscles.peckMuscle(0.001, 35.4, 0.015, 1.097718, 1.410458, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("GH_L", "Geniohyoid_L",               null, FrankMuscles.peckMuscle(0.001, 32.0, 0.015, 1.221939, 1.570069, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("GH_R", "Geniohyoid_R",               null, FrankMuscles.peckMuscle(0.001, 32.0, 0.015, 1.222497, 1.570787, 0.0, 1.0),     false, null, externalMuscDir, externalMuscles);
        loadMuscle("SH_L", "Stylohyoid_L",               null, FrankMuscles.peckMuscle(0.001, 15.6, 0.015, 1.0, 1.284900, 0.0, 1.0),          false, null, externalMuscDir, externalMuscles);
        loadMuscle("SH_R", "Stylohyoid_R",               null, FrankMuscles.peckMuscle(0.001, 15.6, 0.015, 1.0, 1.284900, 0.0, 1.0),          false, null, externalMuscDir, externalMuscles);
        externalExciters = createMuscleExciters("ExternalExciters", externalMuscles, excitersCombineLR);
        //*/
        
        //*/ --- face muscles --- 
        loadMuscle("DAO",   "DepressorAnguliOris",               null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("BUC",   "Buccinator",                        null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("DLI",   "DepressorLabiiInferioris",          null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("MENT",  "Mentalis",                          null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("OOM",   "ObicularisOrisMiddle",              null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("OOP",   "ObicularisOrisPeripheral",          null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("LLSAN", "LevatorLabiiSuperiorisAlaequeNasi", null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("LAO",   "LevatorAnguliOris",                 null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("RIS",   "Risorius",                          null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("ZYG",   "Zygomaticus",                       null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        loadMuscle("LLS",   "LevatorLabiiSuperioris",            null, FrankMuscles.constantMuscle(0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, face, faceMuscDir, faceMuscles);
        faceExciters = createMuscleExciters("FaceExciters", faceMuscles, excitersCombineLR);
        addUpperLowerOO(faceExciters, faceMuscles);
        //*/

        //*/ --- tongue muscles --- 
        // values from Ian Stavness thesis 2011, divided by number of fibres strands in parallel
        loadMuscle("GGA",   "Genioglossus_Anterior",  null, FrankMuscles.peckMuscle(0.0, 32.8/12.0,  0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("GGM",   "Genioglossus_Middle",    null, FrankMuscles.peckMuscle(0.0, 22.0/8.0,   0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("GGP",   "Genioglossus_Posterior", null, FrankMuscles.peckMuscle(0.0, 67.2/24.0,  0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("GH",    "Geniohyoid",             null, FrankMuscles.peckMuscle(0.0, 32.0/12.0,  0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("HG",    "Hyoglossus",             null, FrankMuscles.peckMuscle(0.0, 118.0/20.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("IL",    "InferiorLongitudinal",   null, FrankMuscles.peckMuscle(0.0, 16.4/8.0,   0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("MH",    "Mylohyoid",              null, FrankMuscles.peckMuscle(0.0, 46.8/28.0,  0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("STY",   "Styloglossus",           null, FrankMuscles.peckMuscle(0.0, 43.6/16.0,  0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("TRANS", "Transversus",            null, FrankMuscles.peckMuscle(0.0, 90.8/51.0,  0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("VERT",  "Verticalis",             null, FrankMuscles.peckMuscle(0.0, 36.4/102.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, tongue, tongueMuscDir, tongueMuscles);
        loadMuscle("SL",    "SuperiorLongitudinal",   null, FrankMuscles.peckMuscle(0.0, 34.4/22.0,  0.0, 1.0, 2.0, 0.0, 1.0), false, tongue, tongueMuscDir, tongueMuscles);
        tongueExciters = createMuscleExciters("TongueExciters", tongueMuscles, excitersCombineLR);
        //*/
        
        //*/ --- palate muscles --- 
        // muscle force divided by number of muscle strands
        double lvp_max = (8.8*0.5 * 3.9*0.5)*Math.PI/100.0 * 40.0; // ~10.8 N, from Inouye2015/Perry2013
        double pg_max = Math.pow(3.2*0.5, 2.0)*Math.PI/100.0 * 40.0; // width from cho2013
        double mu_max = (5.9*0.5 * 3.0*0.5)*Math.PI/100.0 * 40.0; //Azzam1977, series1995 has a fairly different value
        double pp_max = 0.48 * 2.0/3.0 * 40.0; //Pearson2012 pcsa reporting palatopharyngeus + salpingopharyngeus where they join; assume 2/3 is PP
        double tvp_max = (10.0*0.5 * 1.5*0.5)*Math.PI/100.0 * 40.0; // a guess from cho2013
        double pf = 0.5;
        double damping = 0.0;
        loadMuscle("LVP", "LevatorVeliPalitini",    null, FrankMuscles.peckMuscle(damping, lvp_max/2.0, pf, 1.0, 2.0, 0.0, 1.0), true, softPalate, palateMuscDir, palateMuscles);
        loadMuscle("MU",  "MusculusUvulae",         null, FrankMuscles.peckMuscle(damping, mu_max/2.0,  pf, 1.0, 2.0, 0.0, 1.0), true, softPalate, palateMuscDir, palateMuscles);
        loadMuscle("PGA", "PalatoglossusAnterior",  null, FrankMuscles.peckMuscle(damping, pg_max/3.0,  pf, 1.0, 2.0, 0.0, 1.0), true, softPalate, palateMuscDir, palateMuscles);
        loadMuscle("PGP", "PalatoglossusPosterior", null, FrankMuscles.peckMuscle(damping, pg_max/3.0,  pf, 1.0, 2.0, 0.0, 1.0), true, softPalate, palateMuscDir, palateMuscles);
        loadMuscle("PP",  "Palatopharyngeus",       null, FrankMuscles.peckMuscle(damping, pp_max/4.0,  pf, 1.0, 2.0, 0.0, 1.0), true, softPalate, palateMuscDir, palateMuscles);
        loadMuscle("TVP", "TensorVeliPalitini",     null, FrankMuscles.peckMuscle(damping, tvp_max/2.0, pf, 1.0, 2.0, 0.0, 1.0), true, softPalate, palateMuscDir, palateMuscles);
        palateExciters = createMuscleExciters("PalateExciters", palateMuscles, excitersCombineLR);
        
        // attach the LVP to all nodes within radius, not just to an element
//        defineMuscleMarkerRadius(softPalate, palateMuscles.get("LVP_R"), 0.0039);
//        defineMuscleMarkerRadius(softPalate, palateMuscles.get("LVP_L"), 0.0039);
        
//        MuscleExciter mex_MU = new MuscleExciter("mex_MusculusUvulae");
//        softPalate.add(mex_MU);
//        defineElementMuscles(softPalate, palateMuscles.get(2), 2.0/1000.0, mex_MU);
//        defineElementMuscles(softPalate, palateMuscles.get(3), 2.0/1000.0, mex_MU);
        //*/
        
        //*/ --- pharynx muscles --- 
        // TODO: define appropriate muscle strengths
        // muscle force divided by number of muscle strands
        loadMuscle("IC1",      "InferiorConstrictor1", null, FrankMuscles.peckMuscle(0.0, 3.0/6.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("IC2",      "InferiorConstrictor2", null, FrankMuscles.peckMuscle(0.0, 3.0/5.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("IC3",      "InferiorConstrictor3", null, FrankMuscles.peckMuscle(0.0, 3.0/4.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("MC1",      "MiddleConstrictor1",   null, FrankMuscles.peckMuscle(0.0, 3.0/4.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("MC2",      "MiddleConstrictor2",   null, FrankMuscles.peckMuscle(0.0, 3.0/4.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("MC3",      "MiddleConstrictor3",   null, FrankMuscles.peckMuscle(0.0, 3.0/4.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("SC1",      "SuperiorConstrictor1", null, FrankMuscles.peckMuscle(0.0, 3.0/3.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("SC2",      "SuperiorConstrictor2", null, FrankMuscles.peckMuscle(0.0, 3.0/3.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("SC3",      "SuperiorConstrictor3", null, FrankMuscles.peckMuscle(0.0, 3.0/3.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
//        loadMuscle("PP1",      "PalatoPharyngeus1",    null, FrankMuscles2.peckMuscle(0.0, 3.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
//        loadMuscle("PP2",      "PalatoPharyngeus2",    null, FrankMuscles2.peckMuscle(0.0, 3.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("CP",       "CricoPharyngeal",      null, FrankMuscles.peckMuscle(0.0, 3.0/6.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("SalP",     "SalpingoPharyngeus",   null, FrankMuscles.peckMuscle(0.0, 3.0/4.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("StyP",     "StyloPharyngeus",      null, FrankMuscles.peckMuscle(0.0, 3.0/6.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        loadMuscle("StyP_low", "StyloPharyngeus_low",  null, FrankMuscles.peckMuscle(0.0, 3.0/6.0, 0.0, 1.0, 2.0, 0.0, 1.0), true, pharynx, pharynxMuscDir, pharynxMuscles);
        pharynxExciters = createMuscleExciters("PharynxExciters", pharynxMuscles, excitersCombineLR);
        //*/
        
        //*/ --- larynx muscles --- 
        loadMuscle("CPR",     "CricothyroidParsRecta",              null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CPO",     "CricothyroidParsOblique",            null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("IAT",     "InterarytenoidTransverse",           null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("IAO_1",   "InterarytenoidOblique_1",            null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("IAO_2",   "InterarytenoidOblique_2",            null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TS",      "ThyrohyoidSuperior",                 null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TI",      "ThyrohyoidInferior",                 null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TE",      "ThyroarytenoidExternal",             null, FrankMuscles.peckMuscle(0.0001, 0.6, 0.2, 0.95, 1.15, 0.1, 0.6) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TV",      "ThyroarytenoidVocalis",              null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("LC",      "LateralCricoarytenoid",              null, FrankMuscles.peckMuscle(0.0001, 1.5, 0.2, 0.95, 1.15, 0.1, 1.5) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("PCO",     "PosteriorCricoarytenoidOblique",     null, FrankMuscles.peckMuscle(0.0001, 1.0, 0.2, 0.95, 1.15, 0.1, 1.0) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("ST",      "Sternothyroid",                      null, FrankMuscles.peckMuscle(0.0001, 0.75, 0.2, 0.95, 1.15, 0.1, 0.75) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("SteH",    "Sternohyoid",                        null, FrankMuscles.peckMuscle(0.0001, 0.5, 0.2, 0.95, 1.15, 0.1, 0.5) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CTL",     "CricotrachealLigament",              null, FrankMuscles.linearMaterial(100.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CTPL",    "CricotrachealPosteriorLigament",     null, FrankMuscles.linearMaterial(100.0, 0.001) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CTAL",    "CricotrachealAnteriorLigament",      null, FrankMuscles.linearMaterial(100.0, 0.001) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CAIL",    "CricoarytenoidInferiorLigament",     null, FrankMuscles.linearMaterial(500.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CAML",    "CricoarytenoidMedialLigament",       null, FrankMuscles.linearMaterial(100.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CALL",    "CricoarytenoidLateralLigament",      null, FrankMuscles.linearMaterial(200.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CAAL",    "CricoarytenoidAnteriorLigament",     null, FrankMuscles.linearMaterial(100.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CAPL",    "CricoarytenoidPosteriorLigament",    null, FrankMuscles.linearMaterial(100.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TEL",     "ThyroepiglotticLigament",            null, FrankMuscles.linearMaterial(50.0, 0.001) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TLL",     "ThyrohyoidLateralLigament",          null, FrankMuscles.linearMaterial(5.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TML",     "ThyrohyoidMedianLigament",           null, FrankMuscles.linearMaterial(10.0, 0.001) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TAM1",    "ThyrohyoidAnterosuperiorMembrane1",  null, FrankMuscles.linearMaterial(5.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TAM2",    "ThyrohyoidAnterosuperiorMembrane2",  null, FrankMuscles.linearMaterial(5.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TPM1",    "ThyrohyoidPosterosuperiorMembrane1", null, FrankMuscles.linearMaterial(5.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("TPM2",    "ThyrohyoidPosterosuperiorMembrane2", null, FrankMuscles.linearMaterial(5.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CLL",     "CricothyroidLateralLigament",        null, FrankMuscles.linearMaterial(5000.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("CML",     "CricothyroidMedianLigament",         null, FrankMuscles.linearMaterial(50.0, 0.001) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("VL",      "VocalLigament",                      null, FrankMuscles.linearMaterial(100.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("HML",     "HyoepiglotticMedianLigament",        null, FrankMuscles.linearMaterial(100.0, 0.001) , false, larynx, larynxMuscDir, larynxMuscles);
        loadMuscle("HLL",     "HyoepiglotticLateralLigament",       null, FrankMuscles.linearMaterial(100.0, 0.001) , true, larynx, larynxMuscDir, larynxMuscles);
        larynxExciters = createMuscleExciters("LarynxExciters", larynxMuscles, excitersCombineLR);
        //*/
        
        //*/ --- ligaments --- 
        //XXX: material props not define properly, just added as geom placeholder
        loadMuscle("PTR", "PterygomandibularRaphe", null, FrankMuscles.linearMaterial(1.0, 0.001), true, null, ligamentDir, ligaments);
        loadMuscle("SHL", "StylohyoidLigament",     null, FrankMuscles.linearMaterial(1.0, 0.001), true, null, ligamentDir, ligaments);
        //*/
        
        // this is pretty messy...
        exciterGroups = new ComponentList[]{faceExciters, tongueExciters, palateExciters, pharynxExciters, larynxExciters, externalExciters};
        ComponentList<ComponentList> allExciters = new ComponentList<ComponentList>(ComponentList.class, "MuscleExciters");
        for (ComponentList<MuscleExciter> mexs : exciterGroups)
            allExciters.add(mexs);
        mechModel.add(allExciters);
    }
    
    void loadMuscle(String shortName, String name, String basename, AxialMaterial muscleMat, boolean bilateral, FemMuscleModel fem, String baseDir, ComponentList<MuscleBundle> muscles)
    {
        String displayName = name;
        if (useShortMuscleNames == true)
            displayName = shortName;
        
        if (basename == null)
            basename = name; // a quick hack if I want to keep filenames & muscle names the same
        
        if (bilateral == true)
        {
            String filename = baseDir + basename + "_R.txt";
            if (fem == null)
            {
                FrankMuscles.addMuscleBundles(muscles, displayName+"_R", filename, muscleMat, MuscleSymmetry.none,  mechModel, fems, rbs);
                FrankMuscles.addMuscleBundles(muscles, displayName+"_L", filename, muscleMat, MuscleSymmetry.ySymm, mechModel, fems, rbs);
            }
            else
            {
                FrankMuscles.addMuscleBundles(fem, displayName+"_R", filename, muscleMat, MuscleSymmetry.none,  mechModel, fems, rbs);
                FrankMuscles.addMuscleBundles(fem, displayName+"_L", filename, muscleMat, MuscleSymmetry.ySymm, mechModel, fems, rbs);
            }
        }
        else
        {
            String filename = baseDir + basename + ".txt";
            if (fem == null)
                FrankMuscles.addMuscleBundles(muscles, displayName, filename, muscleMat, MuscleSymmetry.none,  mechModel, fems, rbs);
            else
                FrankMuscles.addMuscleBundles(fem, displayName, filename, muscleMat, MuscleSymmetry.none,  mechModel, fems, rbs);
        }
        
        /*/ Write muscles to table format
        try 
        {
            String matStr = "";
            String assocModel = "";
            if (fem != null)
                assocModel = fem.getName();
            if (muscleMat instanceof AxialMuscleMaterial)
            {
                AxialMuscleMaterial mat = (AxialMuscleMaterial)muscleMat;
                matStr = String.format("%s | %6.2f | %5.3f | %4.2f | %4.2f | %5.3f | %4.2f | %3.1f ", 
                    mat.getClass().toString(), mat.getMaxForce(), mat.getPassiveFraction(), mat.getOptLength(), mat.getMaxLength(), 
                    mat.getDamping(), mat.getTendonRatio(), mat.getForceScaling());
            }
            else if (muscleMat instanceof LinearAxialMaterial)
            {
                LinearAxialMaterial mat = (LinearAxialMaterial)muscleMat;
                matStr = String.format("%s | %6.2f | %5.3f ", mat.getClass().toString(), mat.getStiffness(), mat.getDamping());
            }
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(outputDir + "muscleTable.txt", true)));
            file.printf("%s | %s | %s | %s | %b \n", name, shortName, matStr, assocModel, bilateral);
            file.close();
        }
        catch(IOException e) 
        {
            e.printStackTrace();
        }
        //*/
    }

    public static void defineMuscleMarkerRadius(FemMuscleModel fem, MuscleBundle mb, double radius)
    {
        ArrayList<FemMarker> markers = new ArrayList<>();
        
        for (Muscle m : mb.getFibres())
        {
            Point[] pnts = {m.getFirstPoint(), m.getSecondPoint()};
            for (Point pnt : pnts)
                if ( (pnt instanceof FemMarker) && (markers.contains(pnt) == false) )
                    markers.add((FemMarker)pnt);
        }
        
        for (FemMarker marker : markers)
        {
            // can I override the initial attachment to 
            if (marker.getAttachment() instanceof PointFem3dAttachment)
            {
                PointFem3dAttachment att = (PointFem3dAttachment)marker.getAttachment();
                ArrayList<FemNode3d> nodes = new ArrayList<>();
                for (FemNode3d node : fem.getNodes())
                    if (marker.getPosition().distance( node.getPosition() ) <= radius)
                        nodes.add(node);
                
                att.setFromNodes(marker.getPosition(), nodes);
            }
             
        }
    }
    
//    public static void defineElementMuscles(FemMuscleModel fem, MuscleBundle mb, double muscleElementDist, MuscleExciter mex)
//    {
//        mb.setFibresActive(true);
//
//        LinkedList<MuscleElementDesc> list = mb.getNewElementsNearFibres (muscleElementDist);
//        for (MuscleElementDesc desc : list)
//        {
//            mb.addElement(desc);
//            mex.addTarget(desc);
//        }
//        mb.computeElementDirections();
//    }
    
    public ComponentList<MuscleExciter> createMuscleExciters(String excitersName, ComponentList<MuscleBundle> muscles, boolean combineLR)
    {
        ComponentList<MuscleExciter> exciters = new ComponentList<>(MuscleExciter.class, excitersName);
        
        for (MuscleBundle mb : muscles)
        {
            MuscleExciter mex = new MuscleExciter();
            if ( (combineLR == true) && ((mb.getName().endsWith("_L") == true) || (mb.getName().endsWith("_R") == true)) )
            {
                String name = null;
                if      (mb.getName().endsWith("_L") == true)
                    name = mb.getName().replace("_L", "");
                else if (mb.getName().endsWith("_R") == true)
                    name = mb.getName().replace("_R", "");
                mex.setName(name);
                if (exciters.get(name) == null)
                {
                    mex.addTarget(muscles.get(name + "_L"));
                    mex.addTarget(muscles.get(name + "_R"));
                    exciters.add(mex);
                }
            }
            else
            {
                mex.setName(mb.getName());
                mex.addTarget(mb);
                exciters.add(mex);
            }
        }
        
        return exciters;
    }
    
    // XXX these should be read in from a file -- need a better mechanism for creating/specifying/loading exciters
    private void addUpperLowerOO(ComponentList<MuscleExciter> exciters, RenderableComponentList<MuscleBundle> muscles){
       exciters.add (addBilateralExciter ("OOM_U", "OOM", muscles, new int[]{0,1,2,3,4,5,6}, Color.MAGENTA));
       exciters.add (addBilateralExciter ("OOM_L", "OOM", muscles, new int[]{7,8,9,10,11,12}, Color.ORANGE));
       exciters.add (addBilateralExciter ("OOP_U", "OOP", muscles, new int[]{0,1,2,3,4,5,6,7,8,9,10,11}, Color.CYAN));
       exciters.add (addBilateralExciter ("OOP_L", "OOP", muscles, new int[]{12,13,14,15,16,17,18,19,20,21,22,23,24}, Color.BLUE));
    }
    
    private MuscleExciter addBilateralExciter(String exName, String bName, ComponentList<MuscleBundle> muscles, int[] idxs, Color c) {
       MuscleExciter ex = new MuscleExciter (exName);
       addTargets (ex, muscles.get (bName+"_L"), idxs, c);
       addTargets (ex, muscles.get (bName+"_R"), idxs, c);
       return ex;
    }
    
    private void addTargets(MuscleExciter ex, MuscleBundle b, int[] idxs, Color c) {
       for (int i : idxs) {
          ex.addTarget (b.getFibres ().get (i));
          RenderProps.setLineColor (b.getFibres ().get (i), c);
       }
    }
    
    public MuscleExciter createExciterFromFibres(String name, MuscleBundle[] muscleBundles, int[] fibreSegments)
    {
        MuscleExciter mex = new MuscleExciter();
        mex.setName(name);
        
        for (MuscleBundle muscleBundle : muscleBundles)
            for (int seg : fibreSegments)
                mex.addTarget( muscleBundle.getFibres().get(seg) );
        
        return mex;
    }
    
    public MuscleExciter createExciterExcludingFibres(String name, MuscleBundle[] muscleBundles, int[] fibreSegments)
    {
        MuscleExciter mex = new MuscleExciter();
        mex.setName(name);
        for (MuscleBundle muscleBundle : muscleBundles)
        {
            for (Muscle fibre : muscleBundle.getFibres())
                mex.addTarget( fibre );
            for (int seg : fibreSegments)
                mex.removeTarget( muscleBundle.getFibres().get(seg) );
        }
        
        return mex;
    }
    
    public ArrayList<MuscleExciter> getAllExciters()
    {
        ArrayList<MuscleExciter> allExciters = new ArrayList<>();
        for (ComponentList<MuscleExciter> mexs : exciterGroups)
            for (MuscleExciter mex : mexs)
                allExciters.add(mex);
        return allExciters;
    }
    
    protected void buildControlPanels()
    {
        //GenericModel.buildControlPanels (this, mechModel, true, true, true);
        //GenericModel.buildFemPanels(this, mechModel, fems, false);
        for (FemMuscleModel model : fems)
        {
            FemControlPanel.createControlPanel(this, model, mechModel);
            //        if (model.getMuscleExciters().size() > 0)
            //            this.addControlPanel( GenericModel.createExciterControls_colored(model.getMuscleExciters(), model.getMuscleBundles()) );
        }
        
        //*/ either make control panels using muscle exciters or muscle bundles  
        for (ComponentList<MuscleExciter> me : exciterGroups)
            this.addControlPanel( GenericModel.createExciterControls(me.getName(), me));
        /*/
        for (ComponentList<MuscleBundle> muscleList : muscles)
            this.addControlPanel(GenericModel.createBundleControls(muscleList));
        //*/
        
        ControlPanel vp = GenericModel.initVisibilityPanel(mechModel);
        //vp.addWidget (new JSeparator());
        GenericModel.addGroupToVisibilityPanel(vp, fems);
        GenericModel.addGroupToVisibilityPanel(vp, rbs);
        //GenericModel.addGroupToVisibilityPanel(vp, mechModel.rigidBodyConnectors());
        GenericModel.addComponentsToVisibilityPanel(vp, mechModel.meshBodies());
        vp.addWidget("FrameMarkers", mechModel.frameMarkers(), "renderProps.visible");
        vp.addWidget("Particles",    mechModel.particles(),    "renderProps.visible");
        vp.addWidget (new JSeparator());
        for (RenderableComponentList<ModelComponent> comp : muscles)
        {
            if (comp.getRenderProps() == null)
                comp.setRenderProps(new RenderProps());
            vp.addWidget(comp.getName(), comp, "renderProps.visible");
        }
        //    vp.addWidget("ExternalMuscles", externalMuscles, "renderProps.visible");
        //    vp.addWidget("Ligaments", ligaments, "renderProps.visible");

        this.addControlPanel(vp);
        //this.addControlPanel(GenericModel.createBundleControls(muscles));
        this.mergeAllControlPanels(true);
    }


   protected void addTrackingController() 
   {
      TrackingController trackingController = new TrackingController(mechModel, "trackingController");
      // mid-sagittal tongue surface nodes from front to back:
      int[] midsagittalNodes = new int[] {926, 919, 912, 873, 869, 908, 859, 927, 928, 846, 847};
      // just pick three of these nodes:
//      int[] targetNodes = new int[] {919, 912, 873};
      int[] targetNodes = new int[] {919, 873, 859};
      
      for (int nodeIdx : targetNodes) {
         trackingController.addMotionTarget(tongue.getByNumber (nodeIdx));
      }

      
      for (MuscleExciter ex : tongue.getMuscleExciters()) {
         trackingController.addExciter(ex);
      }
  
//      trackingController.addRegularizationTerms(/*l2norm*/0.1, /*damping*/0.1);
      trackingController.addL2RegularizationTerm(/*l2norm*/0.01);
      trackingController.setMaxExcitationJump (0.1);
      trackingController.setNormalizeH (true);
      InverseManager.useLegacyNames = true;
      trackingController.createProbesAndPanel (this);
      addController(trackingController);
      
      RenderProps.setPointRadius (tongue, 0.8);
      trackingController.setTargetsPointRadius (1.0);
   }
    
    void loadRigidBody(RigidBody rb, String name, String geometryDir, String filename, double density)
    {        
        PolygonalMesh geom = GenericModel.loadGeometry(geometryDir, filename);
        if (geom == null)
            rb = null;
        else
        {
            //rb = new RigidBody();
            rb.setName(name);
            rb.setMesh(geom, null);
            rb.setDensity(density);
            rb.setFrameDamping(rigidBodyFrameDamping);
            rb.setRotaryDamping(rigidBodyRotaryDamping);
            rbs.add(rb);
        }
    }
    
    public void applyFemStaticNodes(FemModel3d fem, String femMeshName)
    {
        String cacheName = geometryDir + femMeshName + "_staticNodes.txt";
        int[] list = GenericModel.readIntList(cacheName);
        GenericModel.setNodesNondynamic(fem, list);
    }
    
    public void attachFemToRigidBody (FemModel3d fem, RigidBody rb, String femMeshName)
    {
        String cacheName = geometryDir + femMeshName + "_attachedNodes_" + rb.getName() + ".txt";
        GenericModel.attachFemNodesToRigidBody(fem, cacheName, rb, mechModel);
    }

//    public void attachFemToRigidBody (FemModel3d fem, RigidBody rb, double distance, boolean surfaceOnly, NodeCaching nodeCaching, String femMeshName)
//    {   
//        // using node caching does not speed up the load time, and only complicates the code, so skip it. 
//        // The use of femMeshName is to deliberately enforce that the cached nodes are stored with the associated mesh. 
//        String cacheName = geometryDir + femMeshName + "_attachedNodes_" + rb.getName() + ".txt";
//        int[] indices = null;
//        if (nodeCaching == NodeCaching.Load)
//        {
//            indices = GenericModel.readIntList(cacheName);
//        }
//        if (indices == null)
//        {
//            ArrayList<FemNode3d> nodes;
//            if (surfaceOnly == true)
//                nodes = GenericModel.findNodesNearSurface(fem, rb.getSurfaceMesh(), distance, true);
//            else
//                nodes = GenericModel.findNodesInRegion(fem, rb.getSurfaceMesh(), distance, false);
//            indices = GenericModel.getIndicesFromNodes(nodes);
//        }
//        GenericModel.attachFemNodesToRigidBody(fem, indices, rb, mechModel);
//        if (nodeCaching == NodeCaching.Save)
//        {
//            GenericModel.writeIntList(cacheName, indices); 
//        }
//    }

    public SkinMeshBody addCenterlineSkin(String name, String filename)
    {
        // load the centerline and create centerline mesh
        CenterlineLinear cl = new CenterlineLinear();
        //cl.readVertices(geometryDir + "centerlinePoints.csv");
        cl.readVertices(geometryDir + filename);
        cl = (CenterlineLinear)cl.getCenterlineSection(cl.getLength()*0.06, cl.getLength()*0.98); //.98 or .93
        cl.resample(60);
        PolylineMesh centerlineMesh = new PolylineMesh(); 
        for (Point3d p : cl.getVertices())
            centerlineMesh.addVertex(p);
        centerlineMesh.addLine(centerlineMesh.getVertices().toArray(new Vertex3d[0]) ); // define the connectivity
        SkinMeshBody centerlineSkin = new SkinMeshBody();
        centerlineSkin.setName(name);
        centerlineSkin.setMesh(centerlineMesh);
        mechModel.addMeshBody(centerlineSkin);
        
        // define the boundary bodies/weights
        for (FemModel3d fem : new FemModel3d[]{face, tongue, softPalate, larynx, pharynx} )
            centerlineSkin.addFemModel(fem);
        for (RigidBody rb : new RigidBody[]{maxilla, jaw} )
            centerlineSkin.addFrame(rb);
        centerlineSkin.computeWeights();
        
        return centerlineSkin;
    }
    
    public SkinMeshBody addAirwaySkin(String name, String filename) 
    {
        SkinMeshBody airway = new SkinMeshBody();
        addAirwaySkin(airway, name, filename);
        return airway;
    }
    
    public void addAirwaySkin(SkinMeshBody airway, String name, String filename) 
    {
        PolygonalMesh mesh = GenericModel.loadGeometry(geometryDir, filename);
        airway.setMesh(mesh);
        airway.setName(name);
        mechModel.addMeshBody(airway);

        // define the boundary bodies/weights
        for (FemModel3d fem : new FemModel3d[]{face, tongue, softPalate, larynx, pharynx} )
            airway.addFemModel(fem);
        for (RigidBody rb : new RigidBody[]{maxilla, jaw} )
            airway.addFrame(rb);
        airway.computeWeights();
    }

    public void defineRendering()
    {
        double scaling = 1.0/1000.0;
        
        for (FemMuscleModel fem : fems)
        {
            //fem.setSurfaceRendering(SurfaceRender.Shaded);
            fem.getRenderProps().setFaceColor(Color.pink);
            fem.getRenderProps().setFaceStyle(FaceStyle.FRONT_AND_BACK);
            fem.setElementWidgetSize(1.0); // render elements with solids widget

            renderMuscles(fem.getMuscleBundles(), Color.red);

            //fem.getNodes().getRenderProps().setVisible (true);
//            GenericModel.renderStaticNodes   (fem, 0.3*scaling, Color.black);
//            GenericModel.renderAttachedNodes (fem, 0.3*scaling, Color.green);
//            GenericModel.renderFemMarkers    (fem, 0.7*scaling, new Color(0.6f, 0.0f, 0.6f));

        }
        for (RigidBody rb : rbs )
        {
            rb.getRenderProps().setFaceStyle(FaceStyle.FRONT_AND_BACK);
            rb.getRenderProps().setShading(Shading.SMOOTH);
        }

        for (BodyConnector rbc : mechModel.bodyConnectors())
        {
            rbc.getRenderProps().setFaceColor(Color.gray);
            rbc.getRenderProps().setAlpha(0.5);
            rbc.getRenderProps().setVisible(false);
        }

        // uniquely define the color of components
        face.getRenderProps().setFaceColor      ( new Color(0.80f, 0.62f, 0.46f) );
        tongue.getRenderProps().setFaceColor    ( new Color(0.75f, 0.50f, 0.42f) );
        larynx.getRenderProps().setFaceColor    ( new Color(0.95f, 0.60f, 0.48f) );
        softPalate.getRenderProps().setFaceColor( new Color(0.80f, 0.55f, 0.45f) );
        float[] c2 = {0.45f, 0.45f, 0.60f};
        hyoid.getRenderProps().setFaceColor  ( new Color(c2[0]-0.00f, c2[1]-0.00f, c2[2]-0.00f) );
        maxilla.getRenderProps().setFaceColor( new Color(c2[0]+0.15f, c2[1]+0.15f, c2[2]+0.15f) );
        jaw.getRenderProps().setFaceColor    ( new Color(c2[0]-0.15f, c2[1]-0.15f, c2[2]-0.15f) );
        
        //*/ Render muscles by geometry grouping
        //renderMuscles(faceMuscles, Color.pink, true);
        renderMuscles(tongueMuscles, GenericModel.createColors_Reds());
        renderMuscles(palateMuscles, true);
        renderMuscles(pharynxMuscles, GenericModel.createColors_Reds());
        //renderMuscles(larynxMuscles, Color.white, false);
        renderMuscles(externalMuscles, Color.pink);
        renderMuscles(ligaments, Color.black);
        //*/
        
        /*/ Render muscle by excitation grouping
        for (ComponentList<MuscleExciter> muscExciters : exciters)
        {
            for (MuscleExciter mex : muscExciters)
            {
                Vector3d c = new Vector3d();
                c.setRandom (0.0, 1.0);
                Color color = new Color((float)c.x,(float)c.y,(float)c.z);
                for (int a=0; a<mex.numTargets(); a++)
                {
                    RenderProps.setLineColor((Renderable)mex.getTarget(a), color);
                }
            }
        }
        //*/
        
        for (FemMuscleModel fem : fems)
        {
            for (MuscleBundle mb : fem.getMuscleBundles())
            {
                if (mb.getElements().size() > 0)
                {
                    //mb.setDirectionRenderLen(0.7);
                    mb.setElementWidgetSize(0.5);
                    mb.getRenderProps().setFaceColor( mb.getRenderProps().getLineColor() );
                }
            }
        }
        
        // skin rendering
        centerlineSkin.getRenderProps().setLineColor(Color.magenta);
        centerlineSkin.getRenderProps().setVisible(false);
        for (SkinMeshBody aw : new SkinMeshBody[]{airway, airway_tube})
        {
            RenderProps props = aw.getRenderProps ();
            props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
            props.setFaceColor(new Color(0.2f, 0.4f, 1.0f) );
            props.setShading(Renderer.Shading.SMOOTH);
            props.setVisible(false);
        }
        
        //GenericModel.renderStaticNodes  (softPalate, 0.3*scaling, Color.black);
        //GenericModel.renderAttachedNodes(softPalate, 0.3*scaling, Color.green);
        //GenericModel.renderFemMarkers   (softPalate, 0.3*scaling, Color.cyan);
        //GenericModel.renderPoints(pharynx.getNodes(), 0.0002, Color.cyan);
        //GenericModel.renderPoints(pharynx.markers(), 0.0003, Color.magenta);
        //GenericModel.renderStaticNodes  (pharynx, 0.3*scaling, Color.darkGray);
        //GenericModel.renderAttachedNodes(larynx, 0.3*scaling, Color.cyan);
        //GenericModel.renderAttachedNodes(tongue, 0.3*scaling, Color.green);
        
        
        //GenericModel.renderAttachedPoints(mechModel.particles(), 0.5*scaling, Color.green);
        //GenericModel.renderStaticPoints  (mechModel.particles(), 0.5*scaling, Color.black);
        
        /*/ render collisions
        RenderProps.setVisible  (mechModel.getCollisionManager(), true);
        RenderProps.setLineColor(mechModel.getCollisionManager(), Color.green);
        RenderProps.setEdgeColor(mechModel.getCollisionManager(), Color.red);
        //*/
    }
    
    public void renderMuscles(ComponentList<MuscleBundle> muscles, Color[] colors)
    {
        int idx = 0;
        for (MuscleBundle mb : muscles )
        {
            RenderProps.setLineStyle(mb, LineStyle.SPINDLE);
            RenderProps.setLineRadius(mb, 0.0003);
            
            Color color = colors[idx % colors.length];
            if      (mb.getName().endsWith("_L") == true)
            {
                RenderProps.setLineColor(mb, color);
                muscles.get( mb.getName().replace("_L", "_R") ).getRenderProps().setLineColor(color);
                idx++;
            }
            else if (mb.getName().endsWith("_R") == true)
            {
                //muscles.get( mb.getName().replace("_R", "_L") ).getRenderProps().setLineColor(color);
            }
            else
            {
                RenderProps.setLineColor(mb, color);
                idx++;
            }
            
        }
    }
    
    public void renderMuscles(ComponentList<MuscleBundle> muscles, Color color)
    {
        renderMuscles(muscles, new Color[]{color});
    }
    
    public void renderMuscles(ComponentList<MuscleBundle> muscles, boolean randomColors)
    {
        Color[] colors = new Color[muscles.size()];
        Vector3d c = new Vector3d();
        
        for (int i=0; i<colors.length; i++)
        {    
            c.setRandom (0.0, 1.0);
            colors[i] = new Color((float)c.x,(float)c.y,(float)c.z);
        }
        renderMuscles(muscles, colors);
    }
    
    public ArrayList<Marker> createSynthEmmaPoints()
    {
        ArrayList<Marker> markers = new ArrayList<Marker>();
        ArrayList<String> propNames = new ArrayList<>(); 
        // String[] propNames = {"SoftTissues/tongue/markers/1800:position", "frameMarkers/172:position"};
        
        LinkedHashMap<Point3d,FemModel3d> femList = new LinkedHashMap<>();
        femList.put(tongue.getNode(919).getPosition(), tongue);
        femList.put(tongue.getNode(873).getPosition(), tongue);
        femList.put(tongue.getNode(908).getPosition(), tongue);
        femList.put(tongue.getNode(927).getPosition(), tongue);
        femList.put(tongue.getNode(847).getPosition(), tongue); // base of tongue, mid-sag
        
        femList.put(softPalate.getNode(2543).getPosition(), softPalate);
        femList.put(softPalate.getNode(2931).getPosition(), softPalate); // top bulge of palate
        femList.put(softPalate.getNode(114).getPosition(), softPalate);  // bottom of uvula
        femList.put(softPalate.getNode(2582).getPosition(), softPalate); // point in oral cavity 
        
        LinkedHashMap<Point3d,Frame> rbList = new LinkedHashMap<>();
        rbList.put(new Point3d(0.0561215, 0.0, 0.0886421), jaw);
        
        for (Point3d p : femList.keySet())
        {
            Point3d pPert = new Point3d(p); // I get errors if I don't perturb the point position w the soft palate
            pPert.add(1e-7, 1e-7, 1e-7);
            
            FemModel3d model = femList.get(p);
            Marker m = model.addMarker( pPert );
            markers.add(m);
            propNames.add(  String.format("%s/%s/markers/%d:position", fems.getName(), model.getName(), model.markers().indexOf(m))  );
        }
        
        for (Point3d p : rbList.keySet() )
        {
            Frame frame = rbList.get(p);
            Marker m = mechModel.addFrameMarker(frame, p );
            markers.add(m);
            propNames.add( String.format("%s/%d:position", mechModel.frameMarkers().getName(), mechModel.frameMarkers().indexOf(m)) );
        }

        String[] propNamesArr = new String[propNames.size()];
        for (int a=0; a<propNames.size(); a++)
            propNamesArr[a] = propNames.get(a);
        
        NumericOutputProbe op = new NumericOutputProbe(mechModel, propNamesArr, outputDir + "output_marker_all.txt", -1);
        op.setStartTime(0.0);
        if (this.getInputProbes().size() > 0)
            op.setStopTime(this.getInputProbes().get(0).getStopTime());
        this.addOutputProbe( op );
        
        return markers;
    }
    
    public void correctLarynxPos()
    {
        RigidTransform3d trans = new RigidTransform3d(new Vector3d(141.185, 2.90388, 26.8049), new AxisAngle(new Vector3d(0.56, 0.53, 0.62), 2.059));
        RigidTransform3d transInv = new RigidTransform3d(trans);
        transInv.invert();

        mechModel.scaleDistance(1000.0);
        mechModel.transformGeometry(transInv);
        mechModel.scaleDistance(1.0/1000.0); 
        //OK, should be in Moisik space now. The transforms below brings the larynx structs to symmetry
        mechModel.transformGeometry(new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0),     new AxisAngle(new Vector3d(1.0, 0.0, 0.0), 90.0*(Math.PI/180.0) )));
        mechModel.transformGeometry(new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0),     new AxisAngle(new Vector3d(0.0, 0.0, 1.0), 90.0*(Math.PI/180.0) )));
        mechModel.transformGeometry(new RigidTransform3d(new Vector3d(0.0, 0.0, 0.0),     new AxisAngle(new Vector3d(0.0, 1.0, 0.0), -5.0*(Math.PI/180.0) )));
        mechModel.transformGeometry(new RigidTransform3d(new Vector3d(0.14, 0.0, 0.0267), new AxisAngle(new Vector3d(1.0, 0.0, 0.0),  0.0*(Math.PI/180.0) )));
        
        GenericModel.writeAllGeometries(outputDir, mechModel);
        GenericModel.writeAxialSprings(outputDir, mechModel);
    }
    
    private void setLinearFaceRegion(FemMaterial faceMat, FemMaterial faceMatSafe) 
    {
        // TODO: define "find elements by region/proximity" in GenericModel instead
        double dist = 0.01;
        Point3d region = new Point3d(0.056943742, 0.025309368, 0.096349413); // at corner of lips
        //LinearMaterial faceLinMat = new LinearMaterial(10000.0, 0.40);
        for (FemElement3d elem : face.getElements())
        {
            Point3d cent = new Point3d();
            if (cent.y < 0.0)
                cent.y = cent.y*-1.0;
            elem.computeCentroid(cent);
            if (cent.distance(region) < dist)
                elem.setMaterial(faceMatSafe);
            else
                elem.setMaterial(faceMat);
        }
    }

    public void attach (DriverInterface driver)
    {
        super.attach (driver);
        //this.getMainViewer().setBackgroundColor(Color.white);
        
        //*/
        //this.setViewerCenter( new Point3d(0.09,  0.00, 0.10) );
        //this.setViewerEye(    new Point3d(0.09, -0.39, 0.10) );
        //*/
        
        //photoShoot(0.0);
        
        /*/
        GLViewerFrame viewer2frame = Main.getMain().createViewerFrame();
        viewer2frame.setBounds(1000, 0, 600, 800);
        GLViewer viewer2 = viewer2frame.getViewer();
        //viewer2.getCanvas().setBounds(1000, 0, 600, 600);
        //GLViewer viewer2 = new GLViewer(600, 600);
        viewer2.setCenter(new Point3d( 0.14,  0.00, 0.12));
        viewer2.setEye   (new Point3d(-0.25, -0.16, 0.16));
        viewer2.setBackgroundColor(Color.white);
        //*/
    }
    
    int nStep = 0;
    @Override
    public StepAdjustment advance( double t0, double t1, int flags) 
    {
        /*/ for trouble shooting, write out muscle forces...
        for (ComponentList<MuscleBundle> mList : muscles)
            writeMuscleMaxForces(String.format("%sMuscleForces_%s_%f.csv", outputDir, mList.getName(), t0), mList);
        //*/
        
        //photoShoot(t0);
        
        // TODO: implement an OutputMonitor for saving images and geometries
        if (saveFrames == true)
        {
            
            for (int a=0; a<Main.getMain().getViewerManager().numViewers(); a++)
            {
                String imgDir = String.format("%sviewer%d_images/", outputDir, a);
                if (nStep == 0)
                    GenericModel.createDirectory(imgDir);
                saveScreenshot(Main.getMain().getViewerManager().getViewer(a), String.format("%simage_%04d", imgDir, nStep), 2);
            }
            
            
//            GLViewer viewer = this.getMainViewer();
//            File file = new File(String.format ("%simage_%04d.jpg", outputDir, nStep));
//            viewer.setupScreenShot (viewer.getScreenWidth(), viewer.getScreenHeight(), FrameBufferObject.defaultSamples, file, "jpg");
//            viewer.rerender();
//            //viewer.repaint();
//            viewer.paint();
//            //saveScreenshot(viewer, String.format ("%simage_%04d", outputDir, nStep), 2);
//            nStep++;
        }

        if (saveGeometries == true)
        {
            try
            {
                String geomDir = String.format("%sgeometry/", outputDir);
                if (nStep == 0)
                    GenericModel.createDirectory(geomDir);
                
                for (FemModel3d fem : fems)
                    VtkInputOutput.writeVTK(String.format("%sfrank_%s_%05.3f.vtk", geomDir, fem.getName(), t0), fem);
                for (RigidBody rb : new RigidBody[]{hyoid, maxilla, jaw, thyroid, cricoid} )
                    PlyWriter.writeMesh( String.format("%sfrank_%s_%05.3f.ply", geomDir, rb.getName(), t0), rb.getSurfaceMesh() );
                //VtkInputOutput.writeVTK(String.format("%sfrank_%s_%05.3f.vtk", geomDir, rb.getName(), t0), rb.getSurfaceMesh());
                PlyWriter.writeMesh( String.format("%sfrank_%s_%05.3f.ply", geomDir, airway.getName(), t0), airway.getSurfaceMesh() );
                PlyWriter.writeMesh( String.format("%sfrank_%s_%05.3f.ply", geomDir, airway_tube.getName(), t0), airway_tube.getSurfaceMesh() );
                GenericModel.writeVerticesToCSV(String.format("%sfrank_centerline_%05.3f.csv", geomDir, t0), centerlineSkin.getMesh().getVertices());
                //VtkInputOutput.writeVTK(String.format("%sfrank_%s_%05.3f.vtk", geomDir, airway.getName(), t0), airway.getSurfaceMesh());
            }
            catch (Exception e)
            {}
        }
        
        nStep++;
                      
        return super.advance(t0, t1, flags);
        
    }
    
    public void photoShoot(double time)
    {
        mechModel.setDynamicsEnabled(false);
        //larynxMuscles.getRenderProps().setVisible(false);
        externalMuscles.getRenderProps().setVisible(false);
        
        double tEnd = 10.0;
        double tRot = 0.25;
        double dist = 0.5;
        double tRotMod = (time % tRot) * 1.0/tRot;
        Point3d pCenter = new Point3d(0.12, 0.0, 0.1);
        Point3d pCamera = new Point3d(pCenter.x - dist*Math.cos(tRotMod*Math.PI*2.0), pCenter.y - dist*Math.sin(tRotMod*Math.PI*2.0), pCenter.z);
        this.setViewerCenter(pCenter);
        this.setViewerEye(pCamera);

        if (time/tRot >= 1.0)
        {
            face.getRenderProps().setVisible(false);
            face.getMuscleBundles().get("OOP").getRenderProps().setVisible(false);
        }
        if (time/tRot >= 2.0)
        {
            jaw.getRenderProps().setVisible(false);
            maxilla.getRenderProps().setVisible(false);
        }
        if (time/tRot >= 3.0)
        {
            pharynx.getRenderProps().setVisible(false);
            palateMuscles.getRenderProps().setVisible(false);
        }
        if (time/tRot >= 4.0)
            airway.getRenderProps().setVisible(true);
    }
    
    public void saveScreenshot(GLViewer viewer, String filename, float resFactor)
    {
        //GLViewer viewer = this.getMainViewer();
        File file = new File(String.format ("%s.jpg", filename));
        int imgW = Math.round(viewer.getScreenWidth()*resFactor);
        int imgH = Math.round(viewer.getScreenHeight()*resFactor);
        viewer.setupScreenShot (imgW, imgH, -1, file, "jpg");
        //viewer.setupScreenShot (viewer.getScreenWidth(), viewer.getScreenHeight(), FrameBufferObject.defaultSamples, file, "jpg");
        viewer.rerender();
        //viewer.repaint();
        viewer.paint();
    }
    
    public void writeAllMuscles(String saveFolder) 
    {
        String saveDir = outputDir + saveFolder;
        GenericModel.createDirectory(saveDir);
        for (ComponentList<MuscleBundle> muscs : muscles)
        {
            //String muscWriteDir = modelDir + "output/muscles/" + muscs.getName() + "/";
            String muscWriteDir = saveDir + muscs.getName() + "/";
            GenericModel.createDirectory(muscWriteDir);
            FrankMuscles.writeMuscleBundles(muscs, muscWriteDir);
        }
    }
    
    public void writeMuscleMaxForces(String filename, ComponentList<MuscleBundle> muscles)
    {   
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
            file.printf("name,fMin,fMax,fAvg,fAvgPassive\n");
            
            for (MuscleBundle mb : muscles)
            {
                double fMin = mb.getFibres().get(0).getForceNorm();
                double fMax = fMin;
                double fAvg = 0.0;
                double fAvgPassive = 0.0;
                for (Muscle m : mb.getFibres())
                {
                    double f = m.getForceNorm();
                    if ( f > fMax )
                        fMax = f;
                    if (f < fMin )
                        fMin  = f;
                    fAvg = fAvg + f;
                    fAvgPassive = fAvgPassive + m.getPassiveForceNorm();
                }
                fAvg = fAvg/((double)mb.getFibres().size());
                fAvgPassive = fAvgPassive/((double)mb.getFibres().size());
                
                file.printf("%s,%f,%f,%f,%f\n", mb.getName(), fMin, fMax, fAvg, fAvgPassive);
            }
            file.close();

        }
        catch(Exception e)
        {
            //System.out.printf("Error writing muscle bundle %s to %s.\n", mb.getName(), filename);
            e.printStackTrace();
        }
        
        
    }
    
    // --- Create menu items for running an activation sequence --- //
    public boolean getMenuItems(List<Object> items) 
    {
        items.add (GuiUtils.createMenuItem (this, "speech_a", ""));
        items.add (GuiUtils.createMenuItem (this, "speech_i", ""));
        items.add (GuiUtils.createMenuItem (this, "speech_u", ""));
        items.add (GuiUtils.createMenuItem (this, "speech_aiu", ""));
        
        items.add (GuiUtils.createMenuItem (this, "openJaw", ""));
        items.add (GuiUtils.createMenuItem (this, "closeJaw", ""));
        items.add (GuiUtils.createMenuItem (this, "velopharyngealClosure", ""));
        items.add (GuiUtils.createMenuItem (this, "oropharyngealIsthmus", ""));
        items.add (GuiUtils.createMenuItem (this, "face_protrusion", ""));
        return true;
    }

    public void actionPerformed (ActionEvent event) 
    {
        ArrayList<MuscleExciter> allExciters = getAllExciters();
        String cmd = event.getActionCommand();
        if      (cmd.equals ("speech_a"))
            FrankActivations.probe_snd_a(this, mechModel, allExciters);
        else if (cmd.equals ("speech_i"))
            FrankActivations.probe_snd_i(this, mechModel, allExciters);
        else if (cmd.equals ("speech_u"))
            FrankActivations.probe_snd_u(this, mechModel, allExciters);
        else if (cmd.equals ("speech_aiu"))
            FrankActivations.probe_snd_a_i_u(this, mechModel, allExciters);
        
        else if (cmd.equals ("openJaw"))
            FrankActivations.probe_jaw_OpenJaw(this, mechModel, allExciters); // XXX: does not finish
        else if (cmd.equals ("closeJaw"))
            FrankActivations.probe_jaw_CloseJaw(this, mechModel, allExciters);
        else if (cmd.equals ("velopharyngealClosure"))
            FrankActivations.probe_palate_vpClosure(this, mechModel, softPalate);
        else if (cmd.equals ("oropharyngealIsthmus"))
            FrankActivations.probe_palate_opIsthmus(this, mechModel, softPalate);
        else if (cmd.equals ("face_protrusion"))
        {
//            face.setMaterial( new LinearMaterial(15000.0, 0.49) ); // not nice
//            face.setIncompressible (IncompMethod.OFF);
            FrankActivations.makeOOPElemMuscles(FrankActivations.findExciter(allExciters, "OOP"), face);
            FrankActivations.probe_face_Protrusion(this, mechModel, allExciters); // XXX: does not finish
        }
        
    }    
    // --- --- //
    
    /**
     * {@inheritDoc}
     */
    public String getAbout() 
    {
        File about = new File(modelDir + "AboutFrankModel.txt");
        return artisynth.core.util.TextFromFile.getTextOrError(about);
    }

}

   
