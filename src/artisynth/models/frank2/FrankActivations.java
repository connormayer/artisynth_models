package artisynth.models.frank2;

import java.util.ArrayList;

import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.properties.Property;
import maspack.render.RenderProps;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel.IncompMethod;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.MuscleElementDesc;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.workspace.RootModel;
import artisynth.models.fluid1d.fileIO.CSV;

public class FrankActivations 
{
    
    // TODO: add option for exciters vs muscle bundles
    // TODO: generalize createMuscleProbe --> createProbe(component, property, options....)
    // think about muscle groupings: jaw opening: muscles, relative strengths --> it may be nice to apply timing/strenght to muscle groups
    
    public static void probe_palate_vpClosure(RootModel rootModel, MechModel mechModel, FemMuscleModel palate)
    {
        double[] time = {0.0, 0.1, 0.3, 0.4};
        double[] exct = {0.0, 0.0, 0.3, 0.3};
        
        //palate.setMaterial(new LinearMaterial(500.0, 0.49));
        
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_L", time, exct));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_R", time, exct));
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_palate_vpClosure_ie(RootModel rootModel, MechModel mechModel, FemMuscleModel palate)
    {
        double[] time     = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
        double[] exct_ext = {0.0, 0.0, 0.3, 0.3, 0.3, 0.3};
        double[] exct_int = {0.0, 0.0, 0.0, 0.0, 0.3, 0.3};
        
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_ext_L", time, exct_ext));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_ext_R", time, exct_ext));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_int_L", time, exct_int));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_int_R", time, exct_int));
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_palate_opIsthmus_ie(RootModel rootModel, MechModel mechModel, FemMuscleModel palate)
    {
        double[] time     = {0.0, 0.1, 0.3, 0.4, 0.5, 0.55};
        double[] exct_lvp = {0.0, 0.0, 0.3, 0.3, 0.3, 0.3};
        double[] exct_pgp = {0.0, 0.0, 0.0, 0.0, 0.1, 0.1};
        
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_ext_L", time, exct_lvp));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_ext_R", time, exct_lvp));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_int_L", time, exct_lvp));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_int_R", time, exct_lvp));
        
        rootModel.addInputProbe(createMuscleProbe(palate, "PGP_ext_L", time, exct_pgp));
        rootModel.addInputProbe(createMuscleProbe(palate, "PGP_ext_R", time, exct_pgp));
//        rootModel.addInputProbe(createMuscleProbe(palate, "PGP_int_L", time, exct_pgp));
//        rootModel.addInputProbe(createMuscleProbe(palate, "PGP_int_R", time, exct_pgp));
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_palate_opIsthmus(RootModel rootModel, MechModel mechModel, FemMuscleModel palate)
    {
        double[] time     = {0.0, 0.1, 0.3, 0.4, 0.55, 0.7, 0.9, 1.0};
        double[] exct_lvp = {0.0, 0.0, 0.3, 0.3, 0.3,  0.3, 0.0, 0.0};
        double[] exct_pgp = {0.0, 0.0, 0.0, 0.0, 0.1,  0.0, 0.0, 0.0};
        
        //palate.setMaterial(new LinearMaterial(500.0, 0.49));
        
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_L", time, exct_lvp));
        rootModel.addInputProbe(createMuscleProbe(palate, "LVP_R", time, exct_lvp));
        rootModel.addInputProbe(createMuscleProbe(palate, "PGP_L", time, exct_pgp));
        rootModel.addInputProbe(createMuscleProbe(palate, "PGP_R", time, exct_pgp));
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_tongue_simple(RootModel rootModel, MechModel mechModel, RenderableComponentList<MuscleBundle> muscles, FemMuscleModel tongue)
    {
        double[] time = {0.0, 0.02, 0.2};
        double[] exct = {0.0, 0.0, 1.0};
        
        rootModel.getInputProbes().add( createMuscleProbe(tongue, "GGP_R", time, exct, 0.15) );
        rootModel.getInputProbes().add( createMuscleProbe(tongue, "GGP_L", time, exct, 0.15) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_jaw_OpenJaw(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        double[] time = {0.0, 0.1, 0.5};
        double[] act  = {0.0, 0.0, 1.0};
        
        MuscleExciter mex_JO = makeJawOpenExciter(exciters);
        mechModel.addMuscleExciter(mex_JO);
        
        rootModel.addInputProbe( createMuscleProbe(mex_JO, "JawOpen", time, act) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_jaw_CloseJaw(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        double[] time = {0.0, 0.1, 0.5};
        double[] act  = {0.0, 0.0, 0.015};
        
        MuscleExciter mex_JC = makeJawCloseExciter(exciters);
        mechModel.addMuscleExciter(mex_JC);
        
        rootModel.addInputProbe( createMuscleProbe(mex_JC, "JawClose", time, act) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void makeOOPElemMuscles(MuscleExciter me, FemMuscleModel face)
    {
        me.removeAllTargets();
        MuscleBundle oop_r = face.getMuscleBundles().get("OOP_R");
        MuscleBundle oop_l = face.getMuscleBundles().get("OOP_L");
        face.getMuscleBundles().remove(oop_r);
        face.getMuscleBundles().remove(oop_l);
        MuscleBundle oop = FrankMuscles.combineMuscleBundles(oop_r, oop_l);
        oop.setName("OOP");
        face.addMuscleBundle(oop);
        int[] des = {2218, 2217, 2216, 2215, 2214, 2213, 2212, 2211, 2210, 3125, 2209, 3124, 2208, 2207, 2206, 3123, 2138, 2126, 2127, 2128, 2129, 2130, 2131, 2132, 2133, 2134, 2135, 2136, 2137, 5308, 5307, 5306, 5305, 5304, 5303, 5302, 5301, 5300, 5299, 5298, 5297, 5309, 6294, 5377, 5378, 5379, 6295, 5380, 6296, 5381, 5382, 5383, 5384, 5385, 5386, 5387, 5388, 5389};
        for (int id : des) 
        {
            MuscleElementDesc desc = new MuscleElementDesc();
            FemElement3d elem = face.getElements().getByNumber(id-1); // this version of the face is indexed from 0
            desc.setElement(elem);
            oop.addElement(desc);
            // John Lloyd, Jul 13, 2019: just add entire bundle as me target 
            //me.addTarget(desc, 5.0);
        }
        me.addTarget(oop, 5.0);
        oop.computeElementDirections();
        RenderProps.setVisible(oop, true);
        oop.setElementWidgetSize(1.0);
        oop.setDirectionRenderLen(0.7);
    }
    
    public static MuscleExciter makeJawOpenExciter(Iterable<MuscleExciter> exciters)
    {
        MuscleExciter me_JO = new MuscleExciter();
        me_JO.setName("mex_JawOpen");
        me_JO.addTarget(findExciter(exciters, "AD"));
        me_JO.addTarget(findExciter(exciters, "PD"));
        me_JO.addTarget(findExciter(exciters, "SteH"));
        return me_JO;
    }
    
    public static MuscleExciter makeJawCloseExciter(Iterable<MuscleExciter> exciters)
    {
        MuscleExciter me = new MuscleExciter();
        me.setName("mex_JawClose");
        String[] jc = {"AT", "MT", "PT", "DM", "SM", "MP"};
        for (String mName : jc)
            me.addTarget( findExciter(exciters, mName) );
        return me;
    }
    
    public static void probe_snd_ash(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        double[] time = {0.0, 0.1, 0.15, 0.3, 0.35, 0.5};
        //double[] time = {0.0, 0.05, 0.1, 0.15, 0.2, 0.25};
        
        MuscleExciter mex_JO = makeJawOpenExciter(exciters);
        MuscleExciter mex_JC = makeJawCloseExciter(exciters);
        mechModel.addMuscleExciter(mex_JO);
        mechModel.addMuscleExciter(mex_JC);
        
        // now define the sequence...
        rootModel.addInputProbe( createMuscleProbe(mex_JO, "JawOpen",                      time, new double[]{0.00, 0.00, 0.14, 0.14, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe(mex_JC, "JawClose",                     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.015, 0.015}) );
        // tongue activations
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "STY"),  "STY",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGP"),  "GGP",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.25, 0.25}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGM"),  "GGM",  time, new double[]{0.00, 0.00, 0.08, 0.08, 0.1, 0.1}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGA"),  "GGA",  time, new double[]{0.00, 0.00, 0.12, 0.12, 0.02, 0.02}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "HG"),   "HG",   time, new double[]{0.00, 0.00, 0.15, 0.15, 0.00, 0.00}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "VERT"), "VERT", time, new double[]{0.00, 0.00, 0.05, 0.05, 0.00, 0.00}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "TRANS"),"TRANS",time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "SL"), "SL",     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "IL"), "IL",     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "MH"), "MH",     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.10, 0.10}) );
        // face
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "RIS"), "RIS",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "ZYG"), "ZYG",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "OOP"), "OOP",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.50, 0.50}) );
        // palate activations
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "LVP"), "LVP",   time, new double[]{0.00, 0.00, 0.05, 0.05, 0.05, 0.05}) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.01)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_snd_a_i_u(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        //double[] time = {0.0, 0.1, 0.2, 0.4, 0.5, 0.7, 0.8, 1.0};
        double[] time = {0.0, 0.05, 0.1, 0.3, 0.35, 0.55, 0.6, 0.8};
        
        MuscleExciter mex_JO = makeJawOpenExciter(exciters);
        MuscleExciter mex_JC = makeJawCloseExciter(exciters);
        mechModel.addMuscleExciter(mex_JO);
        mechModel.addMuscleExciter(mex_JC);
        
        // now define the sequence...
        //rootModel.addInputProbe( createMuscleProbe(mex_JO, "JawOpen",                      time, new double[]{0.00, 0.00, 0.14, 0.14, 0.00, 0.00, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe(mex_JO, "JawOpen",                      time, new double[]{0.00, 0.00, 0.16, 0.16, 0.00, 0.00, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe(mex_JC, "JawClose",                     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.01, 0.01, 0.01, 0.01}) );
        // tongue activations
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "STY"),  "STY",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.15, 0.15}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGP"),  "GGP",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.50, 0.50, 0.10, 0.10}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGM"),  "GGM",  time, new double[]{0.00, 0.00, 0.08, 0.08, 0.02, 0.02, 0.03, 0.03}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGA"),  "GGA",  time, new double[]{0.00, 0.00, 0.12, 0.12, 0.02, 0.02, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "HG"),   "HG",   time, new double[]{0.00, 0.00, 0.15, 0.15, 0.00, 0.00, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "VERT"), "VERT", time, new double[]{0.00, 0.00, 0.05, 0.05, 0.00, 0.00, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "TRANS"),"TRANS",time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.10, 0.10}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "SL"),   "SL",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05, 0.07, 0.07}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "IL"),   "IL",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "MH"),   "MH",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.10, 0.10, 0.00, 0.00}) );
        // face
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "RIS"),  "RIS",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "ZYG"),  "ZYG",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05, 0.00, 0.00}) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "OOP"),  "OOP",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.40, 0.40}) );
        // palate activations
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "LVP"),  "LVP",  time, new double[]{0.00, 0.00, 0.05, 0.05, 0.10, 0.10, 0.15, 0.15}) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.01)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_snd_u_alt(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        double[] time = {0.0, 0.1, 0.3, 0.4};
        double[] exct = {0.0, 0.0, 1.0, 1.0};
        
        MuscleExciter mex_JC = makeJawCloseExciter(exciters);
        mechModel.addMuscleExciter(mex_JC);
        
        //rootModel.addInputProbe( createMuscleProbe(mex_JC, "JawClose",   time, exct, 0.00) );
        // tongue
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "STY"),   "STY",   time, exct, 0.20) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGP"),  "GGP",  time, exct, 0.03) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGM"),  "GGM",  time, exct, 0.03) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGA"),  "GGA",  time, exct, 0.15) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "VERT"), "VERT", time, exct, 0.15) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "TRANS"),"TRANS",time, exct, 0.10) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "SL"), "SL",     time, exct, 0.02) );
        // face
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "OOP"),   "OOP",   time, exct, 0.45) );
        // palate activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "LVP") , "LVP", time, exct, 0.05) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.01)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_snd_u(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        double[] time = {0.0, 0.1, 0.3, 0.4};
        double[] exct = {0.0, 0.0, 1.0, 1.0};
        
        MuscleExciter mex_JC = makeJawCloseExciter(exciters);
        mechModel.addMuscleExciter(mex_JC);
        
        //rootModel.addInputProbe( createMuscleProbe(mex_JC, "JawClose",   time, exct, 0.00) );
        // tongue
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "STY"),   "STY",   time, exct, 0.15) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGP"),  "GGP",  time, exct, 0.10) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGM"),  "GGM",  time, exct, 0.03) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGA"),  "GGA",  time, exct, 0.15) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "VERT"), "VERT", time, exct, 0.15) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "TRANS"),"TRANS",time, exct, 0.10) );
        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "SL"), "SL",     time, exct, 0.07) );
        // face
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "OOP"),   "OOP",   time, exct, 0.40) );
        // palate activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "LVP") , "LVP", time, exct, 0.15) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.01)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_snd_i(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        // initially based on Stavness2012
        double[] time = {0.0, 0.1, 0.3, 0.4};
        double[] exct = {0.0, 0.0, 1.0, 1.0};
        double scale = 1.0; // 
        
        // genioglossus: posterior(0.5), middle (0.2), anterior (0.2); superior longitudinal (0.05); interior longitudinal (0.05); mylohyoid(0.10)
        // face: RIS,ZYG (0.05)
        // jaw: temporalis (anterior, middle, posterior), masseter, median pterygoid (0.01)
        // tongue muscle
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "GGP"),   "GGP",   time, exct, 0.35) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "GGM"),   "GGM",   time, exct, 0.08) );
        //rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "GGA"),   "GGA",   time, exct, 0.00) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "SL"),    "SL",    time, exct, 0.05) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "IL"),    "IL",    time, exct, 0.20) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "MH"),    "MH",    time, exct, 0.08) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "STY"),   "STY",   time, exct, 0.18) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "TRANS"), "TRANS", time, exct, 0.14) );
        //rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "VERT"),  "VERT",  time, exct, 0.03) );
        
        // jaw muscles
        MuscleExciter me_jc = makeJawCloseExciter(exciters);
        MuscleExciter me_jo = makeJawCloseExciter(exciters);
        //rootModel.getInputProbes().add( createMuscleProbe(me_jc, "JO", time, exct, 0.30) );
        rootModel.getInputProbes().add( createMuscleProbe(me_jc, "JC", time, exct, 0.01) );
        
        // face activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "RIS"),  "RIS",  time, exct, 0.05) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "ZYG"),  "ZYG",  time, exct, 0.05) );
        
        // palate activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "LVP") , "LVP", time, exct, 0.10*scale) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_snd_i_old(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        // initially based on Stavness2012
        double[] time = {0.0, 0.1, 0.3, 0.4};
        double[] exct = {0.0, 0.0, 1.0, 1.0};
        double scale = 1.0; // 
        
        // genioglossus: posterior(0.5), middle (0.2), anterior (0.2); superior longitudinal (0.05); interior longitudinal (0.05); mylohyoid(0.10)
        // face: RIS,ZYG (0.05)
        // jaw: temporalis (anterior, middle, posterior), masseter, median pterygoid (0.01)
        // tongue muscle
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "GGP"),   "GGP",   time, exct, 0.50) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "GGM"),   "GGM",   time, exct, 0.02) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "GGA"),   "GGA",   time, exct, 0.02) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "SL"),    "SL",    time, exct, 0.05) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "IL"),    "IL",    time, exct, 0.05) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "MH"),    "MH",    time, exct, 0.10) );
        //rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "TRANS"), "TRANS", time, exct, 0.03) );
        //rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "VERT"),  "VERT",  time, exct, 0.03) );
        
        // jaw muscles
        MuscleExciter me = new MuscleExciter();
        me.setName("mex_JC");
        mechModel.addMuscleExciter(me);
        String[] jc = {"AT", "MT", "PT", "DM", "SM", "MP"};
        for (String mName : jc)
            me.addTarget( findExciter(exciters, mName) );
        rootModel.getInputProbes().add( createMuscleProbe(me, "JC", time, exct, 0.01) );
        
        // face activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "RIS"),  "RIS",  time, exct, 0.05) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "ZYG"),  "ZYG",  time, exct, 0.05) );
        
        // palate activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(exciters, "LVP") , "LVP", time, exct, 0.10*scale) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    


    public static void probe_snd_a(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> mexs)
    {
        // initially based on Stavness2012
        // Muscles: HG, VERT, GGA
        // all muscles activate with the same pattern, the amplitude is adjusted with exciter gain
//        double[] time = {0.0, 0.1, 0.3, 0.4, 0.6, 0.7};
//        double[] exct = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
        double[] time = {0.0, 0.1, 0.3, 0.4};
        double[] exct = {0.0, 0.0, 1.0, 1.0};
        double scale = 1.0; // 
        
        // Jaw-opening muscles
        MuscleExciter me_JO = makeJawOpenExciter(mexs);
        mechModel.addMuscleExciter(me_JO);
        rootModel.getInputProbes().add( createMuscleProbe(me_JO, "JawOpen", time, exct, 0.16*scale) ); // 0rig 0.04
       
        // tongue activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(mexs, "GGM") ,  "GGM", time, exct, 0.08*scale) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(mexs, "GGA") ,  "GGA", time, exct, 0.12*scale) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(mexs, "HG") ,   "HG",  time, exct, 0.15*scale) );
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(mexs, "VERT") , "VERT",time, exct, 0.05*scale) );
        
        // palate activations
        rootModel.getInputProbes().add( createMuscleProbe( findExciter(mexs, "LVP") , "LVP", time, exct, 0.05*scale) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.01)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    public static void probe_snd_a_wu2014_new(RootModel rootModel, MechModel mechModel, RenderableComponentList<MuscleBundle> muscles, FemMuscleModel fem)
    {
        // Muscles: HG, VERT, GGA
        // all muscles activate with the same pattern, the amplitude is adjusted with exciter gain
        double[] time = {0.0, 0.1, 0.3, 0.4, 0.6, 0.7};
        double[] exct = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0};


        // Jaw-opening muscles
        // Following 4 lines are for debugging only.
        System.out.println("==== jaw muscles ====");
        for (MuscleBundle mb : muscles) {
            System.out.println(mb.getName());
        }

        System.out.println("====================");
        //MuscleExciter me_JO = new MuscleExciter();
        //me_JO.setName("mex_JawOpen");
        //me_JO.addTarget(muscles.get("AD_R"));
        //me_JO.addTarget(muscles.get("AD_L"));
        //me_JO.addTarget(muscles.get("PD_R"));
        //me_JO.addTarget(muscles.get("PD_L"));
        //me_JO.addTarget(muscles.get("right sternohyoid"));
        //me_JO.addTarget(muscles.get("left sternohyoid"));
        //mechModel.addMuscleExciter(me_JO);
        //rootModel.getInputProbes().add( createMuscleProbe(me_JO, "JawOpen", time, exct, 0.20) );

        double scale = 0.2/6.51; // used to scale activation level, bringing strongest level to 0.2
        Object[][] seq = {
                {"GGM",   new String[]{"GGM_R", "GGM_L", "GGP_R", "GGP_L"},   1.75}, //
                {"HG",    new String[]{"HG_R", "HG_L"},                       6.51}, //
                {"STY",   new String[]{"STY_R", "STY_L"},                     3.00}, // 6.04
                // SL
                {"IL",    new String[]{"IL_R", "IL_L"},                       2.00}, // 4.61
                {"VERT",  new String[]{"VERT_R", "VERT_L"},                   1.30}, // 1.30
                //{"TRANS", new String[]{"TRANS_R", "TRANS_L"},                 1.30}, // 1.30
                //GH
                //MG
        };

        for (Object[] obj : seq)
        {
            String name = (String)obj[0];
            String[] muscs = (String[])obj[1];
            double level = (double)obj[2];
            MuscleExciter me = new MuscleExciter();
            me.setName("mex_" + name);
            for (String musc : muscs)
                me.addTarget(fem.getMuscleBundles().get(musc) );
            mechModel.addMuscleExciter(me);
            rootModel.getInputProbes().add( createMuscleProbe(me, name, time, exct, level*scale) );
        }

//        // tongue muscles
//        MuscleExciter me_hg = new MuscleExciter();
//        me_hg.setName("mex_HG");
//        me_hg.addTarget(fem.getMuscleBundles().get("HG_R") );
//        me_hg.addTarget(fem.getMuscleBundles().get("HG_L") );
//        mechModel.addMuscleExciter(me_hg);
//        rootModel.getInputProbes().add( createMuscleProbe(me_hg, "HG", time, exct, 0.30*scale) );
//
//        MuscleExciter me_sg = new MuscleExciter();
//        me_sg.setName("mex_STY");
//        me_sg.addTarget(fem.getMuscleBundles().get("STY_R") );
//        me_sg.addTarget(fem.getMuscleBundles().get("STY_L") );
//        mechModel.addMuscleExciter(me_sg);
//        rootModel.getInputProbes().add( createMuscleProbe(me_sg, "STY", time, exct, 0.30*scale) );
//
//        MuscleExciter me_gga = new MuscleExciter();
//        me_gga.setName("mex_GG");
//        me_gga.addTarget(fem.getMuscleBundles().get("GGM_R") );
//        me_gga.addTarget(fem.getMuscleBundles().get("GGM_L") );
//        me_gga.addTarget(fem.getMuscleBundles().get("GGP_R") );
//        me_gga.addTarget(fem.getMuscleBundles().get("GGP_L") );
//        mechModel.addMuscleExciter(me_gga);
//        rootModel.getInputProbes().add( createMuscleProbe(me_gga, "GG", time, exct, 0.25*scale) );
//
//        MuscleExciter me_vert = new MuscleExciter();
//        me_vert.setName("mex_VERT");
//        me_vert.addTarget(fem.getMuscleBundles().get("VERT_R") );
//        me_vert.addTarget(fem.getMuscleBundles().get("VERT_L") );
//        mechModel.addMuscleExciter(me_vert);
//        rootModel.getInputProbes().add( createMuscleProbe(me_vert, "VERT", time, exct, 0.15*scale) );

        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }

    public static void probe_snd_a_payan2009_new(RootModel rootModel, MechModel mechModel, FemMuscleModel fem)
    {
        // Muscles: HG, VERT, GGA
        // all muscles activate with the same pattern, the amplitude is adjusted with exciter gain
        double[] time = {0.0, 0.1, 0.3, 0.4, 0.6, 0.7};
        double[] exct = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
        double scale = 0.6;

        MuscleExciter me_hg = new MuscleExciter();
        me_hg.setName("mex_HG");
        me_hg.addTarget(fem.getMuscleBundles().get("HG_R") );
        me_hg.addTarget(fem.getMuscleBundles().get("HG_L") );
        mechModel.addMuscleExciter(me_hg);
        rootModel.getInputProbes().add( createMuscleProbe(me_hg, "HG", time, exct, 0.30*scale) );

        MuscleExciter me_gga = new MuscleExciter();
        me_gga.setName("mex_GGA");
        me_gga.addTarget(fem.getMuscleBundles().get("GGA_R") );
        me_gga.addTarget(fem.getMuscleBundles().get("GGA_L") );
        mechModel.addMuscleExciter(me_gga);
        rootModel.getInputProbes().add( createMuscleProbe(me_gga, "GGA", time, exct, 0.25*scale) );

        MuscleExciter me_vert = new MuscleExciter();
        me_vert.setName("mex_VERT");
        me_vert.addTarget(fem.getMuscleBundles().get("VERT_R") );
        me_vert.addTarget(fem.getMuscleBundles().get("VERT_L") );
        mechModel.addMuscleExciter(me_vert);
        rootModel.getInputProbes().add( createMuscleProbe(me_vert, "VERT", time, exct, 0.15*scale) );

        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    public static void probe_snd_a_wu2014(RootModel rootModel, MechModel mechModel, RenderableComponentList<MuscleBundle> muscles, FemMuscleModel fem)
    {
        // Muscles: HG, VERT, GGA
        // all muscles activate with the same pattern, the amplitude is adjusted with exciter gain
        double[] time = {0.0, 0.1, 0.3, 0.4, 0.6, 0.7};
        double[] exct = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
        
        
        // Jaw-opening muscles
        MuscleExciter me_JO = new MuscleExciter();
        me_JO.setName("mex_JawOpen");
        me_JO.addTarget(muscles.get("rad"));
        me_JO.addTarget(muscles.get("lad"));
        me_JO.addTarget(muscles.get("rpd"));
        me_JO.addTarget(muscles.get("lpd"));
        me_JO.addTarget(muscles.get("right sternohyoid"));
        me_JO.addTarget(muscles.get("left sternohyoid"));
        mechModel.addMuscleExciter(me_JO);
        rootModel.getInputProbes().add( createMuscleProbe(me_JO, "JawOpen", time, exct, 0.20) );
       
        double scale = 0.2/6.51; // used to scale activation level, bringing strongest level to 0.2
        Object[][] seq = {
              {"GGM",   new String[]{"GGM_R", "GGM_L", "GGP_R", "GGP_L"},   1.75}, //
              {"HG",    new String[]{"HG_R", "HG_L"},                       6.51}, //
              {"STY",   new String[]{"STY_R", "STY_L"},                     3.00}, // 6.04
              // SL
              {"IL",    new String[]{"IL_R", "IL_L"},                       2.00}, // 4.61
              {"VERT",  new String[]{"VERT_R", "VERT_L"},                   1.30}, // 1.30
              //{"TRANS", new String[]{"TRANS_R", "TRANS_L"},                 1.30}, // 1.30
              //GH
              //MG
        };
        
        for (Object[] obj : seq)
        {
            String name = (String)obj[0];
            String[] muscs = (String[])obj[1];
            double level = (double)obj[2];
            MuscleExciter me = new MuscleExciter();
            me.setName("mex_" + name);
            for (String musc : muscs)
                me.addTarget(fem.getMuscleBundles().get(musc) );
            mechModel.addMuscleExciter(me);
            rootModel.getInputProbes().add( createMuscleProbe(me, name, time, exct, level*scale) );
        }
        
//        // tongue muscles
//        MuscleExciter me_hg = new MuscleExciter();
//        me_hg.setName("mex_HG");
//        me_hg.addTarget(fem.getMuscleBundles().get("HG_R") );
//        me_hg.addTarget(fem.getMuscleBundles().get("HG_L") );
//        mechModel.addMuscleExciter(me_hg);
//        rootModel.getInputProbes().add( createMuscleProbe(me_hg, "HG", time, exct, 0.30*scale) );
//        
//        MuscleExciter me_sg = new MuscleExciter();
//        me_sg.setName("mex_STY");
//        me_sg.addTarget(fem.getMuscleBundles().get("STY_R") );
//        me_sg.addTarget(fem.getMuscleBundles().get("STY_L") );
//        mechModel.addMuscleExciter(me_sg);
//        rootModel.getInputProbes().add( createMuscleProbe(me_sg, "STY", time, exct, 0.30*scale) );
//        
//        MuscleExciter me_gga = new MuscleExciter();
//        me_gga.setName("mex_GG");
//        me_gga.addTarget(fem.getMuscleBundles().get("GGM_R") );
//        me_gga.addTarget(fem.getMuscleBundles().get("GGM_L") );
//        me_gga.addTarget(fem.getMuscleBundles().get("GGP_R") );
//        me_gga.addTarget(fem.getMuscleBundles().get("GGP_L") );
//        mechModel.addMuscleExciter(me_gga);
//        rootModel.getInputProbes().add( createMuscleProbe(me_gga, "GG", time, exct, 0.25*scale) );
//        
//        MuscleExciter me_vert = new MuscleExciter();
//        me_vert.setName("mex_VERT");
//        me_vert.addTarget(fem.getMuscleBundles().get("VERT_R") );
//        me_vert.addTarget(fem.getMuscleBundles().get("VERT_L") );
//        mechModel.addMuscleExciter(me_vert);
//        rootModel.getInputProbes().add( createMuscleProbe(me_vert, "VERT", time, exct, 0.15*scale) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_snd_a_payan2009(RootModel rootModel, MechModel mechModel, FemMuscleModel fem)
    {
        // Muscles: HG, VERT, GGA
        // all muscles activate with the same pattern, the amplitude is adjusted with exciter gain
        double[] time = {0.0, 0.1, 0.3, 0.4, 0.6, 0.7};
        double[] exct = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
        double scale = 0.6;
       
        MuscleExciter me_hg = new MuscleExciter();
        me_hg.setName("mex_HG");
        me_hg.addTarget(fem.getMuscleBundles().get("HG_R") );
        me_hg.addTarget(fem.getMuscleBundles().get("HG_L") );
        mechModel.addMuscleExciter(me_hg);
        rootModel.getInputProbes().add( createMuscleProbe(me_hg, "HG", time, exct, 0.30*scale) );
        
        MuscleExciter me_gga = new MuscleExciter();
        me_gga.setName("mex_GGA");
        me_gga.addTarget(fem.getMuscleBundles().get("GGA_R") );
        me_gga.addTarget(fem.getMuscleBundles().get("GGA_L") );
        mechModel.addMuscleExciter(me_gga);
        rootModel.getInputProbes().add( createMuscleProbe(me_gga, "GGA", time, exct, 0.25*scale) );
        
        MuscleExciter me_vert = new MuscleExciter();
        me_vert.setName("mex_VERT");
        me_vert.addTarget(fem.getMuscleBundles().get("VERT_R") );
        me_vert.addTarget(fem.getMuscleBundles().get("VERT_L") );
        mechModel.addMuscleExciter(me_vert);
        rootModel.getInputProbes().add( createMuscleProbe(me_vert, "VERT", time, exct, 0.15*scale) );
        
        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_AndrewsSwallow(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
    {
        
        double[][] data = CSV.ReadData("/home/peter/projects/FrankModel/data/swallow_activations/computedExcitations_orig.csv");
        
        int nRows = data.length;
        int nCols = data[0].length;
        
        String[] probeOrder = {"GGP", "GGM", "GGA", "STY", "GH", "MH", "HG", "VERT", "TRANS", "IL", "SL"};
        
        double[] time = new double[nRows];
        for (int a=0; a< probeOrder.length; a++)
        {
            double[] exct = new double[nRows];
            for (int b=0; b<nRows; b++)
            {
                if (a==0)
                    time[b] = data[b][0];
                exct[b] = data[b][a+1];
            }
            String probeName = probeOrder[a];
            rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, probeName),  probeName,  time, exct) );
        }

        MuscleExciter mex_JO = makeJawOpenExciter(exciters);
        MuscleExciter mex_JC = makeJawCloseExciter(exciters);
        mechModel.addMuscleExciter(mex_JO);
        mechModel.addMuscleExciter(mex_JC);

        // now define the sequence...
        //rootModel.addInputProbe( createMuscleProbe(mex_JO, "JawOpen",                      time, new double[]{0.00, 0.00, 0.14, 0.14, 0.00, 0.00}) );
        //rootModel.addInputProbe( createMuscleProbe(mex_JC, "JawClose",                     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.015, 0.015}) );
        // tongue activations
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "STY"),  "STY",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGP"),  "GGP",  time, new double[]{0.00, 0.00, 0.00, 0.00, 0.25, 0.25}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGM"),  "GGM",  time, new double[]{0.00, 0.00, 0.08, 0.08, 0.1, 0.1}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "GGA"),  "GGA",  time, new double[]{0.00, 0.00, 0.12, 0.12, 0.02, 0.02}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "HG"),   "HG",   time, new double[]{0.00, 0.00, 0.15, 0.15, 0.00, 0.00}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "VERT"), "VERT", time, new double[]{0.00, 0.00, 0.05, 0.05, 0.00, 0.00}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "TRANS"),"TRANS",time, new double[]{0.00, 0.00, 0.00, 0.00, 0.00, 0.00}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "SL"), "SL",     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "IL"), "IL",     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
//        rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "MH"), "MH",     time, new double[]{0.00, 0.00, 0.00, 0.00, 0.10, 0.10}) );
        // face
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "RIS"), "RIS",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "ZYG"), "ZYG",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.05, 0.05}) );
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "OOP"), "OOP",   time, new double[]{0.00, 0.00, 0.00, 0.00, 0.50, 0.50}) );
        // palate activations
        //rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "LVP"), "LVP",   time, new double[]{0.00, 0.00, 0.05, 0.05, 0.05, 0.05}) );

        for (double t=0.0; t<time[time.length-1]; t=t+0.05)
            rootModel.addWayPoint(t);
        rootModel.addBreakPoint(time[time.length-1]);
    }
    
    public static void probe_face_Closure(RootModel rootModel, FemMuscleModel fem)
    {
       rootModel.getInputProbes().add( createMuscleProbe (fem, "OOM", 
          new double[]{0.0, 0.2, 0.6}, 
          new double[]{0.0, 0.0, 0.3}) ); // 0.3 or 0.15
       rootModel.getInputProbes().add( createMuscleProbe (fem, "MENT", 
          new double[]{0.0, 0.2, 0.6}, 
          new double[]{0.0, 0.0, 0.2}) ); // 0.2 or 0.1 
       rootModel.getInputProbes().add( createMuscleProbe (fem, "RIS", 
          new double[]{0.0, 0.2, 0.6}, 
          new double[]{0.0, 0.0, 0.2}) );  // 0.2 or 0.1
       rootModel.addBreakPoint(0.6);
    }

    public static void probe_face_LabialFeature(RootModel rootModel, FemMuscleModel fem)
    {
        // OOPu, OOPl, OOMu, OOMl, MENT, RIS, LLSAN, LAO, DAO, DLI, BUC, ZYG, intraoral pressure, LLS
        // "i" and "s" refer to inferior and superior, respectively. In the model, it's specified as "l" and "u."

        double[] t = {0.0, 0.2, 0.6}; // all activations can use the same timing...
        double[] actNone = {0.0, 0.0, 0.0};
        
        rootModel.getInputProbes().add( createMuscleProbe(fem.getMuscleExciters().get ("OOPu"), "OOPu", t, new double[]{0.0, 0.0, 0.00}) );
        rootModel.getInputProbes().add( createMuscleProbe(fem.getMuscleExciters().get ("OOPl"), "OOPl", t, new double[]{0.0, 0.0, 0.00}) );
        rootModel.getInputProbes().add( createMuscleProbe(fem.getMuscleExciters().get ("OOMu"), "OOMu", t, new double[]{0.0, 0.0, 0.00}) );
        rootModel.getInputProbes().add( createMuscleProbe(fem.getMuscleExciters().get ("OOMl"), "OOMl", t, new double[]{0.0, 0.0, 0.26}) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "MENT",  t, new double[]{0.0, 0.0, 0.26}) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "RIS",   t, new double[]{0.0, 0.0, 0.26}) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "LLSAN", t, new double[]{0.0, 0.0, 0.36}) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "LLS",   t, new double[]{0.0, 0.0, 0.50}) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "LAO",   t, actNone) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "DAO",   t, actNone) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "DLI",   t, actNone) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "BUC",   t, actNone) );
        rootModel.getInputProbes().add( createMuscleProbe (fem, "ZYG",   t, actNone) );
//        rootModel.getInputProbes().add( createMuscleProbe (fem, "intraoralPressure", 
//           new double[]{0.0, 1.0}, 
//           new double[]{0.0, 0.0}) );

        rootModel.addBreakPoint(t[t.length-1]);
     }
     
     public static void probe_face_LabioDental(RootModel rootModel, FemMuscleModel fem)
     {
        // labiodental appears to be identical to LabialFeature 
        probe_face_LabialFeature(rootModel, fem);
     }
     
     public static void probe_face_Perturbation(RootModel rootModel, FemMuscleModel fem)
     {
        // perturbation appears to be the same as LabialFeature, but with Node perturbations added 
        probe_face_LabialFeature(rootModel, fem);
        // TODO: add node perturbations here...
     }
     
     public static void probe_face_ProtrusionOld(RootModel rootModel, FemMuscleModel fem)
     {
//         rootModel.getInputProbes().add( createMuscleProbe (fem, "OOP", 
//             new double[]{0.0, 0.2, 0.6, 0.7}, 
//             new double[]{0.0, 0.0, 0.4, 0.4}) );
         rootModel.getInputProbes().add( createMuscleProbe (fem, "OOP", 
             new double[]{0.0, 0.1, 0.5}, 
             new double[]{0.0, 0.0, 1.0}) );
         rootModel.addBreakPoint(0.7);
     }
     
     public static void probe_face_Protrusion(RootModel rootModel, MechModel mechModel, ArrayList<MuscleExciter> exciters)
     {
         double[] time = {0.0, 0.1, 0.5};
         double[] act  = {0.0, 0.0, 1.0};
         
         rootModel.addInputProbe( createMuscleProbe( findExciter(exciters, "OOP"), "OOP",   time, act) );
         
         for (double t=0.0; t<time[time.length-1]; t=t+0.05)
             rootModel.addWayPoint(t);
         rootModel.addBreakPoint(time[time.length-1]);
     }
     
     // --------------------------------
     
     // TODO: more general --> ModelComponent, Property
     
     public static MuscleExciter findExciter(Iterable<MuscleExciter> mexs, String name)
     {
         for (MuscleExciter mex : mexs)
             if (mex.getName().equalsIgnoreCase(name) == true)
                 return mex;
         return null;
     }
     
     public static void loadMuscleProbesWithExciter(RootModel rootModel, MechModel mechModel, FemMuscleModel fem, String exName, String[] names, double[] time, double[] activation, double gain)
     {
         
         MuscleExciter me = new MuscleExciter();
         me.setName("mex_" + exName);
         for (String name : names)
             me.addTarget(fem.getMuscleBundles().get(name) );
         mechModel.addMuscleExciter(me);
         rootModel.getInputProbes().add( createMuscleProbe(me, exName, time, activation, gain) );
         //return me;
     }
     
     public static void loadMuscleProbes(RootModel rootModel, FemMuscleModel fem, String[] names, double[] time, double[] activation, double gain)
     {
         for (String name : names)
             rootModel.getInputProbes().add( createMuscleProbe (fem.getMuscleBundles().get(name), name, time, activation, gain) );
     }
     
     public static NumericInputProbe createMuscleProbe(FemMuscleModel fem, String name, double[] time, double[] activation)
     {
        return createMuscleProbe (fem.getMuscleBundles().get(name), name, time, activation);
     }
     
     public static NumericInputProbe createMuscleProbe(FemMuscleModel fem, String name, double[] time, double[] activation, double gain)
     {
        return createMuscleProbe (fem.getMuscleBundles().get(name), name, time, activation, gain);
     }
     
     public static NumericInputProbe createMuscleProbe(ModelComponent modelComponent, String name, double[] time, double[] activation)
     {
         return createMuscleProbe(modelComponent, name, time, activation, 1.0);
     }
     
     public static NumericInputProbe createMuscleProbe(ModelComponent modelComponent, String name, double[] time, double[] activation, double gain)
     {
         NumericInputProbe probe = new NumericInputProbe();
         probe.setName(name);

         Interpolation interp = new Interpolation();
         interp.setOrder(Order.Linear); // TODO: make this an option...
         probe.setInterpolation(interp);

         Property[] props = {modelComponent.getProperty ("excitation")};
         probe.setModelFromComponent(modelComponent);
         probe.setInputProperties(props);
            
         probe.setStartStopTimes(0.0, time[time.length-1]);   // time-axis
         probe.setDefaultDisplayRange(0.0, 1.0);              // y-axis
         
         //probe.setModel(fem);
         //MuscleBundle mb = fem.getMuscleBundles().get(name);

//         Property[] props = {fem.getMuscleBundles().get(name).getProperty("excitation")};
//         String[] driverExpressions = {"V0"};  // ??
//         String[] variableNames = {"V0"};      // ??
//         int[] variableDimensions = {1};
//         probe.set(props, driverExpressions, variableNames, variableDimensions, null);

         for (int a=0; a<time.length; a++)
         {
             probe.addData(new double[] {time[a], activation[a]*gain}, NumericInputProbe.EXPLICIT_TIME);
         }

         return probe;
     }
     // The following lines are added to accommodate time property
    /**
     * Build a 4-point probe time axis: hold at 0 until onset, ramp over
     * rampFraction*duration, then hold until duration.
     */
    public static double[] createOnsetProbeTimes (
            double duration, double onset, double rampFraction) {
        double rampDuration = rampFraction * duration;
        double rampEnd = Math.min (onset + rampDuration, duration);
        return new double[] { 0.0, onset, rampEnd, duration };
    }
    public static double[] createOnsetProbeValues (double excitation) {
        return new double[] { 0.0, 0.0, excitation, excitation };
    }
}
