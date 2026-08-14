package artisynth.models.fluid1d;

import artisynth.models.fluid1d.DESolver;
import artisynth.models.fluid1d.DESolver_Dense;
import artisynth.models.fluid1d.DESolver_Sparse;
import artisynth.models.fluid1d.DESolver.*;

public abstract class FluidSolver_1D_NSE implements FluidSolver_1D
{
    // Solve the 1D, incompressible Navier Stokes

    //
    public DESolver solver  = new DESolver_Sparse();	//solver = new DESolver_Dense();
    //public DESolver solver  = new DESolver_Dense();
    FluidSolution_1D fsN;		// step = n+1 (what we are solving for)
    FluidSolution_1D fs;			// step = n   (most recent solution)
    FluidSolution_1D fs0;		// step = n-1
    FluidSolution_1D fs00;		// step = n-2
    FluidSolution_1D fs000;		// step = n-3

    boolean dummyMode = false;		// set true to ignore the fluid

    double t;				// simulation time
    double dx;				// spatial step size
    int N;				// number of points
    int nVars;				// number of variables
    int nEqns;				// number of discrete eqns (should be N*nVars)
    public boolean verbose = true;

    // BC terms
    public enum BCType {uIn_pIn, uIn_pOut, qIn_pIn, pIn_uOut, pIn_pOut, pIn_pOut_extented, resistor, uIn_pIn_dpLimited, uIn_pOut_dpLimited, mixedUP, unset};
    //uInlet, uOutlet, qInlet, qOutlet, pInlet, pOutlet, rInlet, rOutlet
    BCType bcType;
    BCType bcTypeOriginal = BCType.unset;
    double uInlet;
    double uOutlet;
    double qInlet;
    double qOutlet;
    double pInlet;
    double pOutlet;
    double lInlet;	// length of modeled inlet section
    double lOutlet;
    double rInlet;	// resistance 
    double rOutlet;
    double iInlet;	// inertia
    double iOutlet;
    double uMax;
    double uMin;
    double pMax;
    double pMin;
    double dpMax;
    boolean isInletPLimited = false;
    double inletPMax;
    double inletPMin;

    double pUniform   = 0.0;
    double uUniform   = 0.0;
    double tauUniform = 0.0;

    // Discretization Options
    public enum DDXDerScheme {
        ddx_fore, ddx_fore2, ddx_fore3, ddx_fore4, ddx_fore5, ddx_foreMix4, 
        ddx_back, ddx_back2, ddx_back3, ddx_back4, ddx_back5, ddx_backMix4, 
        ddx_cent, ddx_cent5};
        public enum DDTDerScheme {ddt_back, ddt_back2, ddt_cent, ddt_backMix4};

        protected DDXDerScheme ddxDerSchemeMain = DDXDerScheme.ddx_backMix4;
        protected DDXIntScheme ddxIntSchemeMain = DDXIntScheme.ddx_backMix4;
        protected DDTDerScheme ddtDerSchemeMain = DDTDerScheme.ddt_back2;
        protected DDTIntScheme ddtIntSchemeMain = DDTIntScheme.ddt_back2;

        protected DDXDerScheme ddxDerScheme = ddxDerSchemeMain;	// TODO: make same as schemes above when working...
        protected DDXIntScheme ddxIntScheme = ddxIntSchemeMain;
        protected DDTDerScheme ddtDerScheme = ddtDerSchemeMain;
        protected DDTIntScheme ddtIntScheme = ddtIntSchemeMain;		// back 2 seems about the same as back, cent seems unstable

        // Base Eqn Options
        public enum EquationType {bernoulli, nse, uniform, dummy};
        EquationType equationType = EquationType.nse;

        public enum Equation {bernoulliMass, bernoulliMom, nseMass, nseMom, tau, closed_u, closed_p, closed_tau, dummy, 
            uInlet, uOutlet, qInlet, qOutlet, pInlet, pOutlet, pInletExt, pOutletExt, rInlet, rOutlet, tauBC, unset};
            Equation equation = Equation.unset;

            public enum TransitionType {linear, exponential, sinusoid, tanh};
            TransitionType transitionType = TransitionType.tanh;
            double cWeighting = 10.0;

            boolean useTransients = true;	// "true" means that d/dt terms are included in the calculation
            double cT = 1.0;			// multiplying constant to enforce useTransients
            boolean smoothTransients = false;	// smooth the initial transients
            double tSmooth = 0.2;		// span of smoothing (linear smoothing from 0 to 1 in this duration)
            boolean initializeToZero = false;

            // Viscosity Options
            boolean useFrictionLosses = true;
            boolean useGeometryLosses = false;	// sudden expansion/contraction losses --> not implemented...
            boolean useTauSmall = false;
            public enum ChiTerm {ChiSteady, ChiUnsteady, None};		// defines whether chi also modifies unsteady term
            public enum ChiMethod {ReynoldsSharp, ReynoldsSmooth, Area, Static};		// allows chi to change dynamically
            ChiTerm chiType = ChiTerm.None;
            ChiMethod chiMethod = ChiMethod.Static;

            //double chi = 1.0;			// recovery coeff
            double chiMin = 0.0;
            double chiMax = 1.0;
            double chiMin0 = 0.0;
            double chiMax0 = 1.0;
            //boolean dynamicChi = false;	// allow chiMin to be reduced for a small constriction
            double chiSmallAreaRatio = 0.15;	// chiMin is reduced when % openness of geometry drops below this value
            double chiClosedAreaRatio = 0.05;	// chiMin is reduced when % openness of geometry drops below this value
            //double sfScalar = 1.0;		// the step function constant (should be scaled according to the scale of the function being stepped...)
            //double sfShift = 0.0;		// a small shift of the stepped value to make sure 0 values have chi=1 (not 0.5) 
            boolean writeChi = true;		// adds the chi-term to the fluidSolution (thus will be written)

            double areaClosedLimit = 0.0;	// in theory, A=0 is closed, but in practice A=smallNumber may be more robust
            double areaClosedRatio = 0.01;	// closedAreaLimit = A0 * closedAreaRatio
            double areaSmallLimit;		// defines a "small area" in which  a closing model is applied
            double areaSmallRatio = 0.10;	// areaSmallLimit = A0 * areaSmallRatio
            public enum ClosureModel {Simple, Subsection, Enhanced, CorrectedArea};
            ClosureModel closureModel = ClosureModel.Simple;
            boolean[] isGeomClosed;		// an array of open (false) or closed (true) for each segment of geometry
            boolean hasClosure;			// true if there are any closed locations
            public double[] areaCorr;		// TODO: just a hack here for now

            double cGap = 1.0;			// the coefficient describing gap losses
            String gapName = "";


            public abstract void initializeSpecific();

            // TODO: my method of passing the fluid solutions is pretty clumsy...but might be a lot of work to fix X(n+1) handling
            //public abstract FluidSolution_1D stepSpecific(double dt, FluidSolution_1D X1, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00);
            public abstract void stepSpecific(double dt);

            public void initialize()
            {
                initializeSpecific();

                dx = fsN.getGeometry().getCenterline().getS(1) - fsN.getGeometry().getCenterline().getS(0);
                // if fields exist, don't create them??
                fsN.createField("u", N);
                fsN.createField("p", N);
                fsN.createField("tau", N);
                if (writeChi == true)
                    fsN.createField("chi", N);

                solver.setNumberOfPoints(nEqns);

                fs   = fsN.deepCopy();
                fs0  = fsN.deepCopy();
                fs00 = fsN.deepCopy();
                fs000 = fsN.deepCopy();

                isGeomClosed = new boolean[N];

                // Intuitively, I'd expect initializing to the intransient solution would be better, but this performs much better (particularly for the coupled solver)
                if (initializeToZero == false)
                {
                    boolean smoothTransientsTemp = smoothTransients;
                    smoothTransients = false;
                    for (int a=0; a<25; a++)
                        step(0.005);			// its a hack
                    smoothTransients = smoothTransientsTemp;
                }

                //      if (initializeToZero == false)
                //      {
                //	 boolean useTransientsTemp = useTransients;
                //	 setUseTransients(false);
                //	 //useTransients = false;
                //	 for (int i=0; i<4; i++)
                //	    step(1.0);
                //	 setUseTransients(useTransientsTemp);
                //	 //useTransients = useTransientsTemp;
                //      }

                t = 0.0;

                System.out.println("Fluid Solver Initialized");
            }

            public void step(double dt)
            {
                t = t + dt;

                define_cT();					// define transient smoothing, if applicable
                define_GeometryMetrics(fsN);			// define if geometry is closed
                //define_DynamicChi(fsN.getGeometry());		// optionally reduce chiMin if geometry is nearly closed
                //define_StepFuncConstant(dt, fs, fs0, fs00, fs000);	// calculate the step-function constant...
                //createGeometryReports(fsN.getGeometry());

                if      (dummyMode == true)
                {
                    fsN.setField("u",   new double[N]);
                    fsN.setField("p",   new double[N]);
                    fsN.setField("tau", new double[N]);
                }
                else if (equationType == EquationType.uniform)
                {
                    for (int n=0; n<N; n++)
                    {
                        fsN.setField("u",   n, uUniform);
                        fsN.setField("p",   n, pUniform);
                        fsN.setField("tau", n, tauUniform);
                    }
                }
                else if ( (hasClosure == true) && (closureModel == ClosureModel.Simple) )
                {
                    if      ( (bcType == BCType.uIn_pIn) || (bcType == BCType.qIn_pIn) || (bcType == BCType.uIn_pOut)|| (bcType == BCType.uIn_pIn_dpLimited) || (bcType == BCType.uIn_pOut_dpLimited) )
                        //if      ( (bcType == BCType.uIn_pIn) || (bcType == BCType.qIn_pIn) || (bcType == BCType.uIn_pOut) )
                        stepClosedInflow(dt, fsN);
                    else if ( (bcType == BCType.pIn_pOut) || (bcType == BCType.resistor) )
                        stepClosedPressure(dt, fsN);
                }
                else if ( (hasClosure == true) && (closureModel == ClosureModel.Subsection) )
                {
                    stepClosedSubSolver(dt);
                }
                else if (bcTypeOriginal == BCType.mixedUP)
                {
                    uInlet = uMax*(1.0-(fs.getPressure(0)/pMax)*0.75);
                    stepSpecific(dt);

                    //	 estimateSolution( fsN, fs, fs0, fs00 );
                    //	 double dp = 10.0;
                    //	 int iter = 0;

                    //	 while ( (Math.abs(dp) > 0.001) && (iter < 20) )
                    //	 {
                    //	    pInlet = fsN.getPressure(0);
                    //	    uInlet = weightFunc(pInlet,pMax, pMax/4.0, 0.0, uMax, 4.0, TransitionType.tanh);
                    //	    //uExp[n] = uMax - fs.weightFunc(p[n],pMax/3.0,pMax,0.0,uMax, 5.0, TransitionType.exponential);
                    //	    //uSin[n] = fs.weightFunc(p[n],pMax, pMax/6.0, 0.0, uMax, 5.0, TransitionType.sinusoid);
                    //
                    //	    stepSpecific(dt);			// a uIn,pOut step...will update p[0] in fsN.
                    //	    dp = pInlet - fsN.getPressure(0);
                    //	    iter ++;
                    //	 }
                }
                else
                {
                    stepSpecific(dt);		// the specific implementation of the coupled, decoupled, or other solver

                    // now check if the pressure exceeds the pressure limits at the inlet
                    if (isInletPLimited == true)
                    {
                        //	    if (t > 0.68)
                        //	    {t=t;}

                        if (bcType != bcTypeOriginal)
                        {
                            if ( ( (uInlet >= 0.0) && (fsN.getVelocity(0) > uInlet) ) || ( (uInlet < 0.0) && (fsN.getVelocity(0) < uInlet) ) )
                                //if ( ( (uInlet >= 0.0) && (fsN.getVelocity(0) > uInlet) ) || ( (uInlet < 0.0) && (fsN.getVelocity(0) > uInlet) ) )
                            {
                                System.out.println("Switching to velocity BCs");
                                bcType = bcTypeOriginal;
                                stepSpecific(dt);	// restep
                            }
                        }
                        else if (bcType == bcTypeOriginal)
                        {
                            //if ( ( (uInlet >= 0.0) && (fsN.getPressure(0) > inletPMax) ) || ( (uInlet < 0.0) && (fsN.getPressure(0) < inletPMax) ) )
                            if ( (uInlet >= 0.0) && (fsN.getPressure(0) > inletPMax) )
                            {
                                // if velocity is negative, the max pressure should be negative (a min pressure)
                                System.out.println("Switching to pressure BCs");
                                pInlet = inletPMax;
                                bcType = BCType.pIn_pOut;
                                stepSpecific(dt);	// restep
                            }
                            else if ( (uInlet < 0.0) && (fsN.getPressure(0) < inletPMin) )
                            {
                                // if velocity is negative, the max pressure should be negative (a min pressure)
                                System.out.println("Switching to pressure BCs");
                                pInlet = inletPMin;
                                bcType = BCType.pIn_pOut;
                                stepSpecific(dt);	// restep
                            }
                        }
                        // --------------------------------
                        // here is the old way
                        //	    boolean restep = false;
                        //	    if       (fsN.getPressure(0) > inletPMax)
                        //	    {
                        //	       pInlet = inletPMax;
                        //	       restep = true;
                        //	    }
                        //	    else if (fsN.getPressure(0) < inletPMin)
                        //	    {
                        //	       pInlet = inletPMin;
                        //	       restep = true;
                        //	    }
                        //	    if (restep == true)
                        //	    {
                        //	       System.out.println("Switching to pressure BCs");
                        //	       BCType bcOrig = bcType;
                        //	       bcType = BCType.pIn_pOut;
                        //	       stepSpecific(dt);
                        //	       bcType = bcOrig;
                        //	    }
                        // end old way
                    }
                }

                double uMax = max(fsN.getVelocity());
                double[] Re = calculateReynolds(fsN);
                double courant = uMax*dt/dx;
                System.out.printf("Re_min = %f,  Re_max = %f,  Re_avg = %f,  Courant Number = %f \n", min(Re), max(Re), average(Re), courant);

                fs000 = fs00.deepCopy();
                fs00  = fs0.deepCopy();
                fs0   = fs.deepCopy();
                fs    = fsN.deepCopy();			// Better to do this at the beginning, but that messes up how area function is handled
            }

            public void stepClosedSubSolver(double dt)
            {
                // initialize the fields to 0 in fsN (just to be safe)
                fsN.setField("u",   new double[N]);
                fsN.setField("p",   new double[N]);
                fsN.setField("tau", new double[N]);
                estimatePressureBCs();		// update pInlet and pOutlet values

                // Copy global values that need to be remembered: fs*, N, BCs (type and values)
                FluidSolution_1D X1  = fsN;
                FluidSolution_1D X   = fs;
                FluidSolution_1D X0  = fs0;
                FluidSolution_1D X00 = fs00;
                BCType bcOrig = bcType;
                //double pInletOrig = pInlet;
                //double pOutletOrig = pOutlet;
                double uInletOrig = uInlet;
                double uOutletOrig = uOutlet;
                int nOrig = N;

                // 1) define upstream portion
                // find the subset region
                int iStart = 0;
                int iEnd   = 0;
                while (X1.getGeometry().getArea(iEnd) > areaClosedLimit)
                    iEnd++;

                // define the new fs* values as a subset of the full solution      
                fsN  = X1.getSolutionSection(iStart, iEnd);
                fs   = X.getSolutionSection(iStart, iEnd);
                fs0  = X0.getSolutionSection(iStart, iEnd);
                fs00 = X00.getSolutionSection(iStart, iEnd);
                N = fsN.getNumberOfPoints();

                // define BCs: pIn_uOut (pInlet = current p value, uOutlet = 0)
                //setBCs_pINuOUT(fsN.getPressure(0), 0.0);		// set subsection BCs
                setBCs_pINuOUT(pInlet, 0.0);
                stepSubSolver(dt);				// solve the subsection
                X1.setSolutionSection(iStart, fsN);		// copy subset back to full solution

                // --- define downstream portion --- //
                N = X1.getNumberOfPoints();
                iStart = N-1;
                iEnd=N-1;
                while (X1.getGeometry().getArea(iStart) > areaClosedLimit)
                    iStart--;

                // define the new fs* values as a subset of the full solution      
                fsN  = X1.getSolutionSection(iStart, iEnd);
                fs   = X.getSolutionSection(iStart, iEnd);
                fs0  = X0.getSolutionSection(iStart, iEnd);
                fs00 = X00.getSolutionSection(iStart, iEnd);
                N = fsN.getNumberOfPoints();

                // define BCs: pIn_uOut (pInlet = current p value, uOutlet = 0)
                //setBCs_uINpOUT(0.0, fsN.getPressure(N-1));	// set subsection BCs
                setBCs_uINpOUT(0.0, pOutlet);
                stepSubSolver(dt);				// solve the subsection
                X1.setSolutionSection(iStart, fsN);		// copy subset back to full solution

                // --- now back-define everything --- //
                fsN  = X1;
                fs   = X;
                fs0  = X0;
                fs00 = X00;
                bcType  = bcOrig;
                //pInlet  = pInletOrig;
                //pOutlet = pOutletOrig;
                uInlet  = uInletOrig;
                uOutlet = uOutletOrig;
                N = nOrig;

            }

            public void stepSubSolver(double dt)
            {
                // make copies of objects that need to be changed
                DESolver solverOrig = solver;
                int nEqnsOrig = nEqns;

                // redefine values
                N = fsN.getNumberOfPoints();
                nEqns = nVars*N;
                solver = new DESolver_Dense();
                solver.setNumberOfPoints(nEqns);

                stepSpecific(dt);

                nEqns = nEqnsOrig;
                solver = solverOrig;
            }

            public static double[] calculateReynolds(FluidSolution_1D X)
            {
                int N = X.getNumberOfPoints();
                double[] A = X.getGeometry().getArea();
                double[] s = X.getGeometry().getPerimeter();
                double[] u = X.getVelocity();
                double rho = X.getDensity();
                double mu  = X.getViscosity();

                double dHyd = 0.0;
                double[] Re = new double[N];
                for (int n=0; n<N; n++)
                {
                    dHyd  = 4.0*A[n]/s[n];			// hydraulic diameter
                    Re[n] = Math.abs(rho*u[n]*dHyd/mu);	// Reynolds number
                }
                return Re;
            }

            public double[] calculateChi(double dt, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00, FluidSolution_1D X000)
            {
                double rho    = X.getDensity();
                double[] u    = X.getVelocity();
                double[] u0   = X0.getVelocity();
                double[] u00  = X00.getVelocity();
                double[] u000 = X000.getVelocity();

                double[] chi = new double[N];

                if (chiType != ChiTerm.None)
                {
                    if       (chiMethod == ChiMethod.Area)
                    {
                        // In an iterative solution this term only needs to be defined once per time step (area won't change), but organizationally this fits best here
                        double gor = geomOpenRatio(X.getGeometry());
                        if (gor < chiSmallAreaRatio)
                        {
                            double m = (chiMin0 - 0.0)/(chiSmallAreaRatio - chiClosedAreaRatio);
                            chiMin = m*(gor-chiClosedAreaRatio);
                            //chiMin = chiMin0*(gor/chiDampingAreaRatio);
                        }
                        else
                        {
                            chiMin = chiMin0;
                        }
                        //System.out.println("chiMin = " + chiMin);
                    }
                    else if (chiMethod == ChiMethod.ReynoldsSharp)
                    {
                        // Derived from Smith 2004 --> Pressure Recovery in Radiused...
                        // The function is a best-fit to the r/h = 0.833 with chi = 0.2 set as endpoints for Re=0 and Re=25000

                        double re = average(calculateReynolds(X)); // find Re --> thus average Re  (or I could use min Re)
                        if (re > 25000.0)
                            chiMin = 0.20;
                        else
                            chiMin = - 1.479e-22*Math.pow(re, 5.0) + 1.291e-17*Math.pow(re, 4.0) - 4.517e-13*Math.pow(re, 3.0) + 7.588e-09*Math.pow(re, 2.0) - 5.153e-05*re + 0.2033;
                        //System.out.printf("Re_average = %f, \t chiMin = %f \t", re, chiMin);
                    }
                    else if (chiMethod == ChiMethod.ReynoldsSmooth)
                    {
                        // Derived from Smith 2004 --> Pressure Recovery in Radiused...
                        // The function is a best-fit to the r/h = 2.000 with chi = 0.0 at Re=0 and chi = 0.3 at Re=15000
                        double reHigh = 15000.0;
                        double chiHigh = 0.30;
                        double re = average(calculateReynolds(X)); // find Re --> thus average Re  (or I could use min Re)
                        if (re >= reHigh)
                            chiMin = chiHigh;
                        else
                        {
                            chiMin = -chiHigh/Math.pow(reHigh, 2.2) * Math.pow(reHigh-re, 2.2) + chiHigh;
                        }
                        //System.out.printf("Re_average = %f, \t chiMin = %f \t", re, chiMin);
                    }
                    for (int n=0; n<N; n++)
                    {
                        defineDiffSchemes(n,2);

                        //double var1 = u[n]*(rho*ddt(u, u0, u00, dt,n)*cT + rho*u[n]*ddx(u,dx,n));
                        //double var2 = u[n]*(-ddx(p,dx,n));

                        double tSpace = rho*u[n]*ddx(u,dx,n);
                        double tTime  = rho*ddt(u, u0, u00, u000, dt,n)*cT;
                        //chi[n] = (chiMax-chiMin) * stepSmooth(u[n]*(tSpace + tTime)-sfShift, sfScalar) + chiMin;
                        chi[n] = (chiMax-chiMin) * stepDiscrete(u[n]*(tSpace + tTime)) + chiMin;
                    }
                }

                if (writeChi == true)
                    X.setField("chi", chi);

                return chi;
            }


            void defineSchemes(int n, int block)
            {
                defineEquations(n, block);
                defineDiffSchemes(n, block);
            }

            public void defineEquations(int n, int block)
            {
                if (equationType == EquationType.dummy)
                {
                    equation = Equation.dummy;
                }
                else if (n==0)		// set a BC
                {
                    // TODO: this BCs block can certainly be condensed
                    if ( (block == 0) || (block == 1) )
                    {
                        if ( (bcType == BCType.uIn_pIn) || (bcType == BCType.uIn_pIn_dpLimited) )
                        {
                            if      (block == 0)
                                equation = Equation.uInlet;
                            else if (block == 1)
                                equation = Equation.pInlet;
                        }
                        else if ( (bcType == BCType.uIn_pOut) || (bcType == BCType.uIn_pOut_dpLimited) )
                        {
                            if      (block == 0)
                                equation = Equation.uInlet;
                            else if (block == 1)
                                equation = Equation.pOutlet;
                        }
                        else if (bcType == BCType.pIn_pOut)
                        {
                            if      (block == 0)
                                equation = Equation.pInlet;
                            else if (block == 1)
                                equation = Equation.pOutlet;
                        }
                        else if (bcType == BCType.pIn_pOut_extented)
                        {
                            if      (block == 0)
                                equation = Equation.pInletExt;
                            else if (block == 1)
                                equation = Equation.pOutletExt;
                        }
                        else if (bcType == BCType.pIn_uOut)
                        {
                            if      (block == 0)
                                equation = Equation.uOutlet;
                            else if (block == 1)
                                equation = Equation.pInlet;
                        }
                        else if (bcType == BCType.qIn_pIn)
                        {
                            if      (block == 0)
                                equation = Equation.qInlet;
                            else if (block == 1)
                                equation = Equation.pInlet;
                        }
                        else if (bcType == BCType.resistor)
                        {
                            if      (block == 0)
                                equation = Equation.rInlet;
                            else if (block == 1)
                                equation = Equation.rOutlet;
                        }
                    }
                    else if (block == 2)
                    {
                        equation = Equation.tauBC;
                    }
                }
                else
                {
                    if ( (isGeomClosed[n] == true) && (closureModel == ClosureModel.Enhanced) )
                    {
                        if       (block == 0)
                            equation = Equation.closed_u;
                        else if (block == 1)
                            equation = Equation.closed_p;
                        else if (block == 2)
                            equation = Equation.closed_tau;
                    }
                    else
                    {
                        if      ( (equationType == EquationType.bernoulli) && (block == 0) ) 
                            equation = Equation.bernoulliMass;
                        else if ( (equationType == EquationType.bernoulli) && (block == 1) ) 
                            equation = Equation.bernoulliMom;
                        else if ( (equationType == EquationType.nse) && (block == 0) ) 
                            equation = Equation.nseMass;
                        else if ( (equationType == EquationType.nse) && (block == 1) ) 
                            equation = Equation.nseMom;
                        else if ( (block == 2) ) 
                            equation = Equation.tau;
                    }
                }
            }

            public void defineDiffSchemes(int n, int block)
            {
                // --- define the derivative schemes --- //
                if      (n == 0)
                {
                    ddxDerScheme = DDXDerScheme.ddx_fore5;
                    ddxIntScheme = DDXIntScheme.ddx_fore5;		// 1st
                }
                else if (n == 1)
                {
                    ddxDerScheme = DDXDerScheme.ddx_foreMix4;
                    ddxIntScheme = DDXIntScheme.ddx_foreMix4;	// 2nd
                }
                else if (n == N-2)
                {
                    ddxDerScheme = DDXDerScheme.ddx_backMix4;
                    ddxIntScheme = DDXIntScheme.ddx_backMix4;	// 2nd to last
                }
                else if (n == N-1)
                {
                    ddxDerScheme = DDXDerScheme.ddx_back5;
                    ddxIntScheme = DDXIntScheme.ddx_back5;		// last
                }

                //      else if ( (isGeomClosed[n] == false) && (isGeomClosed[n-1] == false) &&  (isGeomClosed[n-2] == true) && (closureModel == ClosureModel.Enhanced) )
                //      {  // near closed section
                //	 ddxDerScheme = DDXDerScheme.ddx_foreMix4;
                //	 ddxIntScheme = DDXIntScheme.ddx_foreMix4;
                //      }
                //      else if ( (isGeomClosed[n] == false) && (isGeomClosed[n-1] == true) && (closureModel == ClosureModel.Enhanced) )
                //      {  // next to closed section
                //	 if (block == 1)
                //	 {
                //	    ddxDerScheme = DDXDerScheme.ddx_foreMix4;
                //	    ddxIntScheme = DDXIntScheme.ddx_foreMix4;
                //	 }
                //	 else
                //	 {
                //	    ddxDerScheme = DDXDerScheme.ddx_fore5;
                //	    ddxIntScheme = DDXIntScheme.ddx_fore5;
                //	 }
                //      }
                //      else if ( (isGeomClosed[n] == false) && (isGeomClosed[n+1] == true) && (closureModel == ClosureModel.Enhanced) )
                //      {  // next to closed section
                //	 //if ( (block == 1) || (block == 0) )
                //	 if (block == 1)
                //	 {
                //	    ddxDerScheme = DDXDerScheme.ddx_backMix4;
                //	    ddxIntScheme = DDXIntScheme.ddx_backMix4;
                //	 }
                //	 else
                //	 {
                //	    ddxDerScheme = DDXDerScheme.ddx_back5;
                //	    ddxIntScheme = DDXIntScheme.ddx_back5;
                //	 }
                //      }
                //      else if ( (isGeomClosed[n] == false) && (isGeomClosed[n+1] == false) &&  (isGeomClosed[n+2] == true) && (closureModel == ClosureModel.Enhanced) )
                //      {  // near closed section
                //	 ddxDerScheme = DDXDerScheme.ddx_backMix4;
                //	 ddxIntScheme = DDXIntScheme.ddx_backMix4;
                //      }

                else
                {
                    ddxIntScheme = ddxIntSchemeMain;
                    ddxDerScheme = ddxDerSchemeMain;
                }
                ddtDerScheme = ddtDerSchemeMain;
                ddtIntScheme = ddtIntSchemeMain;
            }

            public void estimateSolution(FluidSolution_1D Xest, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00, FluidSolution_1D X000)
            {
                // take a guess at the n+1 solution
                double dtFake = 1.0;
                for (int n=0; n<N; n++)
                {
                    Xest.setField("u",   n, X.getField("u",n)   + ddt(X.getField("u"),   X0.getField("u"),   X00.getField("u"),   X000.getField("u"),   dtFake, n)*dtFake*1.0 );
                    Xest.setField("p",   n, X.getField("p",n)   + ddt(X.getField("p"),   X0.getField("p"),   X00.getField("p"),   X000.getField("p"),   dtFake, n)*dtFake*1.0 );
                    Xest.setField("tau", n, X.getField("tau",n) + ddt(X.getField("tau"), X0.getField("tau"), X00.getField("tau"), X000.getField("tau"), dtFake, n)*dtFake*1.0 );
                }
            }

            // --- Weighting functions defined below --- //

            public double weightFunc(double x, double x0, double x1)
            {
                return weightFunc(x,x0,x1, cWeighting, transitionType);
            }

            public double weightFuncDDX(double x, double x0, double x1)
            {
                return weightFuncDeriv(x,x0,x1, cWeighting, transitionType);
            }

            public static double weightFunc(double x, double x0, double x1, double f0, double f1, double c, TransitionType type)
            {
                // beware if x is outside of range x0..x1
                double y = (1.0/(x1-x0))*x - x0/(x1-x0);		// y = x rescaled from 0..1
                double w = 0.0;					// weight function, from 0..1

                if       (type == TransitionType.linear)
                    w = weightLinear(y);
                else if (type == TransitionType.exponential)
                    w = weightExponential(y, c);
                else if (type == TransitionType.sinusoid)
                    w = weightSinusoidal(y);
                else if (type == TransitionType.tanh)
                    w = weightTanh(y, c);

                double f = (f1-f0)*w + f0;			// f rescales w from 0..1 to f0..f1

                return f;
            }

            public static double weightFuncDeriv(double x, double x0, double x1, double f0, double f1, double c, TransitionType type)
            {
                // returns the derivative of the weighting function
                double y    = (1.0/(x1-x0))*x - x0/(x1-x0);
                double dydx =  1.0/(x1-x0);

                double dwdy = 0.0;
                double dfdw = f1-f0;

                if       (type == TransitionType.linear)
                    dwdy = weightLinearDDX(y);
                else if (type == TransitionType.exponential)
                    dwdy = weightExponentialDDX(y, c);
                else if (type == TransitionType.sinusoid)
                    dwdy = weightSinusoidalDDX(y);
                else if (type == TransitionType.tanh)
                    dwdy = weightTanhDDX(y, c);

                double dfdx = dwdy*dydx*dfdw;

                return dfdx;
            }

            public static double weightFunc(double x, double x0, double x1, double c, TransitionType type)
            {
                return weightFunc(x, x0, x1, 0.0, 1.0, c, type);
            }

            public static double weightFuncDeriv(double x, double x0, double x1, double c, TransitionType type)
            {
                return weightFuncDeriv(x, x0, x1, 0.0, 1.0, c, type);
            }

            public static double weightLinear(double x)
            {
                if      (x < 0.0)
                    return 0.0;
                else if (x > 1.0)
                    return 1.0;
                else
                    return x;
            }

            public static double weightExponential(double x, double c)
            {
                // catch exception if x>1??
                return Math.exp(-c*(1.0-x));
            }

            public static double weightSinusoidal(double x)
            {
                if      (x < 0.0)
                    return 0.0;
                else if (x > 1.0)
                    return 1.0;
                else
                    return 0.5+0.5*Math.cos( Math.PI*(x-1.0) );
            }

            public static double weightTanh(double x, double c)
            {
                return 0.5+0.5*Math.tanh( c*(x-0.5) );
            }

            // derivatives
            public static double weightLinearDDX(double x)
            {
                if      (x < 0.0)
                    return 0.0;
                else if (x > 1.0)
                    return 0.0;
                else
                    return 1.0;
            }

            public static double weightExponentialDDX(double x, double c)
            {
                return c*Math.exp(-c*(1.0-x));
            }

            public static double weightSinusoidalDDX(double x)
            {
                if      (x < 0.0)
                    return 0.0;
                else if (x > 1.0)
                    return 0.0;
                else
                    return -0.5*Math.PI*Math.sin( Math.PI*(x-1.0) );
            }

            public static double weightTanhDDX(double x, double c)
            {
                return 0.5*c*(1.0 - Math.pow( Math.tanh(c*(x-0.5)), 2.0) );
                //return 0.5*c/Math.pow( Math.cosh(c*(x-0.5)) ,2.0);		// also true...
            }

            // --- end weighting functions --- //

            public int[] findOpenLoHi(int n)
            {
                int nLo = n;
                while ( (isGeomClosed[nLo] == true) && (nLo >= 0) )
                    nLo--;
                int nHi = n;
                while ( (isGeomClosed[nHi] == true) && (nHi < N) )
                    nHi++;
                return new int[] {nLo, nHi};
            }

            public void closedFunction(double[] F, int n, int block, FluidSolution_1D X, double[] f)
            {
                //closedFunction1(F,n,block,X.getGeometry().getCenterline().getS(), f);
                //closedFunction2(F,n,block,X, f);
            }

            public void closedFunctionJac(DESolver s, int n, int block, FluidSolution_1D X, double[] f)
            {
                //closedFunction1Jac(s,n,block,X.getGeometry().getCenterline().getS(), f);
                //closedFunction2Jac(s,n,block,X, f);
            }

            public void closedFunction1(double[] F, int n, int block, double[] x, double[] f)
            {
                int[] lh = findOpenLoHi(n);
                int nLo = lh[0];
                int nHi = lh[1];
                F[block*N+n] = f[n] - (f[nHi] - f[nLo])*weightFunc(x[n], x[nLo], x[nHi]) - f[nLo];
            }

            public void closedFunction1Jac(DESolver s, int n, int block, double[] x, double[] f)
            {
                int[] lh = findOpenLoHi(n);
                int nLo = lh[0];
                int nHi = lh[1];

                double w = weightFunc(x[n], x[nLo], x[nHi]);
                s.scalarTerm(block*N+n, block*N+n,    1.0);
                s.scalarTerm(block*N+n, block*N+nLo,  w-1.0);
                s.scalarTerm(block*N+n, block*N+nHi, -w);
            }

            public void closedFunction2(double[] F, int n, int block, FluidSolution_1D X, double[] f)
            {
                int[] lh = findOpenLoHi(n);
                int nLo = lh[0];
                int nHi = lh[1];

                double[] x = X.getGeometry().getCenterline().getS();
                double[] A = X.getGeometry().getArea();

                double m = 1.3;
                double divA = Math.pow(A[n], m-1.0)/Math.pow(areaClosedLimit ,m);	// goes to 0 rather than inf

                double w = weightFunc(x[n], x[nLo], x[nHi]);
                F[block*N+n] = f[n] - divA*(w*(f[nHi]*A[nHi] - f[nLo]*A[nLo]) + f[nLo]*A[nLo]);
            }

            public void closedFunction2Jac(DESolver s, int n, int block, FluidSolution_1D X, double[] f)
            {
                int[] lh = findOpenLoHi(n);
                int nLo = lh[0];
                int nHi = lh[1];

                double[] x = X.getGeometry().getCenterline().getS();
                double[] A = X.getGeometry().getArea();

                double m = 1.3;
                double divA = Math.pow(A[n], m-1.0)/Math.pow(areaClosedLimit ,m);	// goes to 0 rather than inf

                double w = weightFunc(x[n], x[nLo], x[nHi]);
                s.scalarTerm(block*N+n, block*N+n,    1.0);
                s.scalarTerm(block*N+n, block*N+nLo,  A[nLo]*divA*(w-1.0));
                s.scalarTerm(block*N+n, block*N+nHi, -A[nHi]*divA*w);
            }


            public double stepDiscrete(double x)
            {
                // a discrete step function
                if (x < 0.0)
                    return 0.0;
                else
                    return 1.0;
            }

            public static double stepSmooth(double x, double cSharp)
            {	
                // TODO: this should be converted to the weight function methods which are generalized much nicer... 
                // Roughly 1 for a positive number and 0 for a negative number
                return Math.atan(cSharp*x)/Math.PI + 0.5;
            }

            public static double[] stepSmooth(double[] x, double cSharp)
            {
                // Roughly 1 for a positive number and 0 for a negative number
                int N = x.length;
                double[] sf = new double[N];

                for (int n=0; n<N; n++)
                    sf[n] = stepSmooth(x[n ], cSharp);

                return sf;
            }

            /*//
   public void define_StepFuncConstant(double dt, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00, FluidSolution_1D X000)
   {  
      double rho    = X.getDensity();
      double[] u    = X.getVelocity();
      double[] p    = X.getPressure();
      double[] u0   = X0.getVelocity();
      double[] u00  = X00.getVelocity();
      double[] u000 = X000.getVelocity();
      double[] var = new double[N];

      for (int n=0; n<N; n++)
      {
	 defineSchemes(n,0);
	 // -dp/dx = rho*du/dt + rho*u*du/dx in the inviscid case
	 var[n] = u[n]*(rho*ddt(u, u0, u00, u000, dt,n)*cT + rho*u[n]*ddx(u,dx,n));
	 //var[n] = u[n]*(-ddx(p,dx,n));
      }

      double range = max(var) - min(var);
      if (range < 0.000000000001)
      {
	 sfShift = 0.001;
	 sfScalar = 100000.0;
      }
      else
      {
	 sfShift = 0.02*range;
	 sfScalar = 10000.0/range;
      }
   }
   //*/

            public void createGeometryReports(Geometry_1D geom)
            {
                double a0 = geom.getArea(0);
                double s0 = geom.getPerimeter(0);

                double aMin = a0;
                double sMin = s0;
                double soaMin = sMin/aMin;
                double soaMax = sMin/aMin;
                double aosMin = aMin/sMin;
                double aosMax = aMin/sMin;

                int nClosures = 0;

                for (int a=0; a<geom.getNumberOfPoints(); a++)
                {
                    double A = geom.getArea(a);
                    double S = geom.getPerimeter(a);
                    if (A/S < aosMin)
                        aosMin = A/S;
                    if (A/S > aosMax)
                        aosMax = A/S;
                    if (S/A < soaMin)
                        soaMin = S/A;
                    if (S/A > soaMax)
                        soaMax = S/A;
                    if (A < aMin)
                        aMin = A;
                    if (S < sMin)
                        sMin = S;

                    if (isGeomClosed[a] == true)
                        nClosures++;
                }

                //System.out.println(String.format("aMin = %f \t, sMin = %f \t", aMin, sMin));
                //System.out.println(String.format("a/sMin = %f \t, a/sMax = %f", aosMin, aosMax));
                //System.out.println(String.format("s/aMin = %f \t, s/aMax = %f", soaMin, soaMax));
                //System.out.println("s/a nonDim = " + (sMin/s0)/(aMin/a0) );
                System.out.printf("num closed = %d", nClosures);
                System.out.println();
            }

            void define_GeometryMetrics(FluidSolution_1D fs)
            {
                // update the "small" and "closed" areas (based on the given ratios)...only will change if A(0,t) changes
                double[] A = fs.getGeometry().getArea();
                double[] s = fs.getGeometry().getPerimeter(); 
                double A0 = A[0];
                //int N = fs.getNumberOfPoints();
                areaClosedLimit = areaClosedRatio*A0;
                areaSmallLimit  = areaSmallRatio*A0;

                // find if the geometry has any "closures"
                int nClosed = 0;
                int nSmall = 0;
                for (int n=0; n<N; n++)
                {
                    if (A[n] <= areaSmallLimit)
                    {
                        nSmall++;
                    }

                    if (A[n] <= areaClosedLimit)
                    {
                        isGeomClosed[n] = true;
                        hasClosure = true;
                        nClosed++;
                    }
                    else
                    {
                        isGeomClosed[n] = false;
                    }
                }

                String geomState = "Open";
                if (nSmall > 0)
                    geomState = "Small";
                if (nClosed > 0)
                    geomState = "Closed";
                System.out.printf("Geometry is %s.   nOpen=%d, nSmall=%d, nClosed=%d \n", geomState, N-nSmall, nSmall-nClosed, nClosed);

                if (closureModel == ClosureModel.CorrectedArea)
                {
                    if (areaCorr == null)
                        areaCorr = new double[N];

                    fs.setField("areaReal", artisynth.models.fluid1d.fluidUtils.ArrayMath.deepCopy_doubleArray(A));
                    double periSmallLimit  = Math.sqrt(4.0*Math.PI*areaSmallLimit);
                    double periClosedLimit = Math.sqrt(4.0*Math.PI*areaClosedLimit);
                    for (int n=0; n<N; n++)
                    {
                        if (A[n] < 0.0)
                            A[n] = 0.0;
                        if (s[n] < 0.0)
                            s[n] = 0.0;
                        //	    A[n] = A[n] + weightFunc(A[n], 0.0, areaSmallLimit, areaClosedLimit, 0.0, 1.5, TransitionType.tanh);
                        //	    s[n] = s[n] + weightFunc(s[n], 0.0, periSmallLimit, periClosedLimit, 0.0, 1.5, TransitionType.tanh);
                        A[n] = A[n] + weightFunc(A[n], 0.0, areaSmallLimit, areaClosedLimit, 0.0, 1.5, TransitionType.linear);
                        s[n] = s[n] + weightFunc(s[n], 0.0, periSmallLimit, periClosedLimit, 0.0, 1.5, TransitionType.linear);
                    }
                }
            }

            void estimatePressureBCs()
            {
                if      (bcTypeOriginal == BCType.uIn_pIn_dpLimited)
                {
                    // pInlet_limited --> define pressure outlet based on dp
                    if (uInlet >= 0.0)
                        pOutlet = pInlet - dpMax;
                    else
                        pOutlet = pInlet + dpMax;

                }
                else if (bcTypeOriginal == BCType.uIn_pOut_dpLimited)
                {
                    // pOutlet_limited --> define pressure inlet based on dp
                    if (uInlet >= 0.0)
                        pInlet = pOutlet + dpMax;
                    else
                        pInlet = pOutlet - dpMax;
                }
                else if ( (bcType == BCType.uIn_pIn) || (bcType == BCType.qIn_pIn) )
                {
                    // pInlet_unlimited --> define pressure outlet based on last time step
                    pOutlet = fsN.getPressure(N-1);
                }
                else if (bcType == BCType.uIn_pOut)
                {
                    // pOutlet_unlimited --> define pressure inlet based on last time step
                    pInlet = fsN.getPressure(0);
                }
            }

            public FluidSolution_1D stepClosedInflow(double dt, FluidSolution_1D X)
            { 
                estimatePressureBCs();      
                return stepClosedPressure(dt, X);
            }

            public FluidSolution_1D stepClosedPressure(double dt, FluidSolution_1D X)
            {
                X.setField("u", new double[N]);		// velocity is 0 everywhere (appx)
                X.setField("p", new double[N]);
                X.setField("tau", new double[N]);
                int a=0;
                //while (X.getGeometry().getArea(a) > closedAreaLimit)
                while (isGeomClosed[a] == false)
                {
                    X.setField("p", a, pInlet);
                    a++;
                }
                a=N-1;
                //while (X.getGeometry().getArea(a) > closedAreaLimit)
                while (isGeomClosed[a] == false)
                {
                    X.setField("p", a, pOutlet);
                    a--;
                }

                return X;
            }

            public FluidSolution_1D stepClosedInflowOld(double dt, FluidSolution_1D X)
            {
                // TODO: this should be improved...// this is a tricky case.  I think pressure should be uniformly increased based on free volume

                X.setField("u",   new double[N]);		// velocity is 0 everywhere (appx)
                X.setField("tau", new double[N]);
                X.setField("p",   new double[N]);		// p1/rho1 = p2/rho2; rho2 = rho1 + massIn/V
                int a=0;
                while (X.getGeometry().getArea(a) > areaClosedLimit)
                {
                    X.setField("p", a, pInlet);
                    a++;
                }

                return X;
            }

            double geomOpenRatio(Geometry_1D geom)
            {
                return min(geom.getArea())/geom.getArea()[0];
            }

            void define_cT()
            {
                double cTmax = 1.00;
                if ( (smoothTransients == true) && (useTransients == true) )
                    if (t/tSmooth < cTmax)
                        cT = t/tSmooth;
                    else
                        cT = cTmax;
            }

            public void setInitialTransientSmoothing(boolean useSmoothing, double duration)
            {
                smoothTransients = useSmoothing;
                tSmooth = duration;
            }

            // --- Define Boundary Conditions --- //
            public void setBCs_uINpIN(double uInlet, double pInlet)
            {
                bcType = BCType.uIn_pIn;
                this.uInlet = uInlet;
                this.pInlet = pInlet;
            }

            public void setBCs_uINpOUT(double uInlet, double pOutlet)
            {
                bcType = BCType.uIn_pOut;
                this.uInlet = uInlet;
                this.pOutlet = pOutlet;
            }

            public void setBCs_qINpIN(double qInlet, double pInlet)
            {
                // Q is the volumetric flow rate m^3/s
                bcType = BCType.qIn_pIn;
                this.qInlet = qInlet;
                this.pInlet = pInlet;
            }

            public void setBCs_pINuOUT(double pInlet, double uOutlet)
            {
                bcType = BCType.pIn_uOut;
                this.pInlet = pInlet;
                this.uOutlet = uOutlet;
            }

            public void setBCs_Pressure(double pInlet, double pOutlet)
            {
                bcType = BCType.pIn_pOut;
                this.pInlet = pInlet;
                this.pOutlet = pOutlet;
            }

            public void setBCs_PressureExtended(double pInlet, double pOutlet, double lInlet, double lOutlet)
            {
                bcType = BCType.pIn_pOut_extented;
                this.pInlet = pInlet;
                this.pOutlet = pOutlet;
                this.lInlet = lInlet;
                this.lOutlet = lOutlet;
            }

            public void setBCs_Resistance(double pInlet, double pOutlet, double rInlet, double rOutlet, double iInlet, double iOutlet)
            {
                bcType = BCType.resistor;
                this.pInlet = pInlet;
                this.pOutlet = pOutlet;
                this.rInlet = rInlet;
                this.rOutlet = rOutlet;
                this.iInlet = iInlet;
                this.iOutlet = iOutlet;
            }

            public void setBCs_Breathing(double qMax, double pMax, double f)
            {
                // q = qMax*sin(2*pi*f*t), but if pOutlet exceeds pMax, then becomes pressure drop
            }

            public void setBCs_uINpOUT_Limited(double uInlet, double pOutlet, double dpMax)
            {
                bcTypeOriginal = BCType.uIn_pOut_dpLimited;
                bcType = BCType.uIn_pOut_dpLimited;
                this.uInlet = uInlet;
                this.pOutlet = pOutlet;
                this.dpMax = dpMax;

                setBCs_inletPLimits(-dpMax, dpMax);

            }

            public void setBCs_inletPLimits(double pMin, double pMax)
            {
                isInletPLimited = true;
                inletPMax = pMax;
                inletPMin = pMin;
            }

            public void setBCs_mixedUPContraints(double uMaxInlet, double pMaxInlet, double pOutlet)
            {
                bcType = BCType.uIn_pOut;
                bcTypeOriginal = BCType.mixedUP;

                this.uMax  = uMaxInlet;
                this.pMax  = pMaxInlet;
                this.pOutlet = pOutlet;
            }

            // --- define loss terms --- //

            double frictionFactor(double Re, double dHyd, double ep)
            {
                double W=weighting(Re);

                double fLam = 64.0/Re;
                double fTurb = Math.pow(1.0/(-1.8*Math.log10(6.9/Re + Math.pow(ep/dHyd/3.7,1.11))), 2.0);
                double f = fLam*W + (1-W)*fTurb;		// interpolated

                return f;
            }

            double weighting(double Re)
            {
                // It might be better to make W smooth, continuous
                double Re1 = 2300;	// orig --> 2300
                double Re2 = 4300;	// orig --> 4300
                double W=0;
                if (Re < Re1)
                    W=1.0;
                else if (Re > Re2)
                    W=0.0;
                else
                    W=1.0-(Re-Re1)/(Re2-Re1);

                return W;
            }

            /*
   public void setRoughness(double roughness)
   {
      //ep = roughness;
   }

   double reynoldsNumber(int a)
   {
      return (soln.rho/soln.mu)*soln.u[a]*(hydraulicDiameter(a));
   }
//*/      

            // --- Some setters ---


            public void setSolution(FluidSolution_1D fluidSolution)
            {
                fsN = fluidSolution;
            }

            public FluidSolution_1D getSolution() 
            {
                return fsN;
            }

            public void setEqn_NSE()
            {
                equationType = EquationType.nse;
            }

            public void setEqn_Bernoulli()
            {
                equationType = EquationType.bernoulli;
                setUseTransients(false);
            }

            public void setEqn_Uniform(double u, double p, double tau)
            {
                equationType = EquationType.uniform;
                uUniform = u;
                pUniform = p;
                tauUniform = tau;
            }

            public void setSchemeDDX(DDXDerScheme ddxDerScheme, DDXIntScheme ddxIntScheme)
            {
                this.ddxDerSchemeMain = ddxDerScheme;
                this.ddxIntSchemeMain = ddxIntScheme;
            }

            public void setSchemeDDT(DDTDerScheme ddtDerScheme, DDTIntScheme ddtIntScheme)
            {
                this.ddtDerSchemeMain = ddtDerScheme;
                this.ddtIntSchemeMain = ddtIntScheme;
                this.ddtDerScheme = ddtDerScheme;
                this.ddtIntScheme = ddtIntScheme;
            }

            public void setUseTransients(boolean useTransients)
            {
                this.useTransients = useTransients;
                if (useTransients == true)
                    cT = 1.0;
                else
                    cT = 0.0;
            }

            public void setUseGeometryLosses(boolean useGeometryLosses)
            {
                this.useGeometryLosses = useGeometryLosses;
            }

            public void setUseFrictionLosses(boolean useFrictionLosses)
            {
                this.useFrictionLosses = useFrictionLosses;
            }

            public void setUseSmallAreaLosses(boolean useTauSmall, double smallAreaRatio)
            {
                this.useTauSmall = useTauSmall;
                this.areaSmallRatio = smallAreaRatio;
            }

            //   public void setUseChiDynamic(boolean useChi)
            //   {
            //      if (useChi == true)
            //	 chiType = ChiTerm.ChiUnsteady;
            //      else
            //	 chiType = ChiTerm.ChiSteady;
            //   }

            public void setChiTerm(ChiTerm chiTerm)
            {
                this.chiType = chiTerm;
            }

            void setChi(double chiMin)
            {
                setChi(chiMin, 1.0);
            }

            void setChi(double chiMin, double chiMax)
            {
                this.chiMin = chiMin;
                this.chiMax = chiMax;
                this.chiMin0 = chiMin;
                this.chiMax0 = chiMax;
            }

            public void setChiMethod_Static(double chiMin)
            {
                chiMethod = ChiMethod.Static;
                setChi(chiMin, 1.0);
            }

            public void setChiMethod_Static(double chiMin, double chiMax)
            {
                chiMethod = ChiMethod.Static;
                setChi(chiMin, chiMax);
            }

            public void setChiMethod_Area(double chiMin, double chiMax, double smallAreaRatio, double closedAreaRatio)
            {
                chiMethod = ChiMethod.Area;
                chiSmallAreaRatio = smallAreaRatio;
                chiClosedAreaRatio = closedAreaRatio;
                setChiMethod_Static(chiMin, chiMax);
            }

            public void setChiMethod_ReynoldsSharp()
            {
                chiMethod = ChiMethod.ReynoldsSharp;
            }

            public void setChiMethod_ReynoldsSmooth()
            {
                chiMethod = ChiMethod.ReynoldsSmooth;
            }

            public void setClosedAreaRatio(double car)
            {
                areaClosedRatio = car;
            }

            public void setClosureModel(ClosureModel cm)
            {
                closureModel = cm;
            }

            public void setClosureModel_Simple(double aClosedRatio)
            {
                setClosedAreaRatio(aClosedRatio);
                setClosureModel(ClosureModel.Simple);
            }

            public void setClosureModel_Subsection(double aClosedRatio)
            {
                setClosedAreaRatio(aClosedRatio);
                setClosureModel(ClosureModel.Subsection);
            }

            public void setClosureModel_AreaCorrection(double aSmallRatio, double aClosedRatio)
            {
                setUseSmallAreaLosses(true, aSmallRatio);
                setClosedAreaRatio(aClosedRatio);
                setClosureModel(ClosureModel.CorrectedArea);
            }

            public void setGapTerm(String gapName, double cGap)
            {
                this.cGap = cGap;
                this.gapName = gapName;
            }

            public void setInitializeToZero(boolean zeroInit)
            {
                initializeToZero = zeroInit;
            }

            public void setDummyMode(boolean dummy)
            {
                dummyMode = dummy;
            }

            public boolean isSteady()
            {
                if (useTransients == false)
                    return true;
                else
                    return false;
            }


            // --- some math helpers --- //	// TODO: move these to a math utils class (or static import??)
            public static double[] add(double[] arr1, double[] arr2)
            {
                if (arr1.length == arr2.length)
                {
                    int N = arr1.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = arr1[n] + arr2[n];
                    return res;
                }
                else
                    return null;
            }

            public static double[] add(double scalar, double[] arr1)
            {
                int N = arr1.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = arr1[n] + scalar;
                return res;

            }

            public static double[] add(double[] arr1, double scalar)
            {
                int N = arr1.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = arr1[n] + scalar;
                return res;

            }

            public static double[] sub(double[] arr, double scalar)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = arr[n] - scalar;
                return res;
            }

            public static double[] sub(double scalar, double[] arr)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = scalar - arr[n];
                return res;
            }

            public static double[] sub(double[] arr2, double[] arr1)
            {
                if (arr1.length == arr2.length)
                {
                    int N = arr1.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = arr2[n] - arr1[n];
                    return res;
                }
                else
                    return null;
            }

            public static double[] mult(double[] arr1, double[] arr2)
            {
                if (arr1.length == arr2.length)
                {
                    int N = arr1.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = arr1[n] * arr2[n];
                    return res;
                }
                else
                    return null;
            }

            public static double[] mult(double scalar, double[] arr)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = arr[n] * scalar;
                return res;
            }

            public static double[] div(double[] arr1, double[] arr2)
            {
                if (arr1.length == arr2.length)
                {
                    int N = arr1.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = arr1[n] / arr2[n];
                    return res;
                }
                else
                    return null;
            }

            public static double[] div(double scalar, double[] arr)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = scalar / arr[n];
                return res;
            }

            public static double[] div(double[] arr, double scalar)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = arr[n] / scalar;
                return res;
            }

            public static double[] scale(double scale, double[] arr)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = scale*arr[n];

                return res;

            }

            public static double[] pow(double[] arr, double power)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = Math.pow(arr[n], power);

                return res;
            }

            public static double ddx(DDXDerScheme ddxScheme, double[] arr, double dx, int i)
            {
                if      (ddxScheme == DDXDerScheme.ddx_fore)
                    return ddx_fore(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_fore2)
                    return ddx_fore2(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_fore3)
                    return ddx_fore3(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_fore4)
                    return ddx_fore4(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_fore5)
                    return ddx_fore5(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_foreMix4)
                    return ddx_foreMix4(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_back)
                    return ddx_back(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_back2)
                    return ddx_back2(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_back3)
                    return ddx_back3(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_back4)
                    return ddx_back4(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_back5)
                    return ddx_back5(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_backMix4)
                    return ddx_backMix4(arr,dx, i);
                else if (ddxScheme == DDXDerScheme.ddx_cent5)
                    return ddx_cent5(arr,dx, i);
                else
                    return ddx_cent2(arr,dx, i);		// the default??
            }

            public double ddx(double[] arr, double dx, int i)
            {
                // TODO: I need to fix this...it overrides my findSchemes!!
                // provides an override at the boundary of the arrays...not carefully checking though!
                //      if      (i == 0)
                //	 return ddx_fore5(arr, dx, i);
                //      else if (i == 1)
                //	 return ddx_foreMix4(arr, dx, i);
                //      else if (i == arr.length-2)
                //	 return ddx_backMix4(arr, dx, i);
                //      else if (i == arr.length-1)
                //	 return ddx_back5(arr, dx, i);
                //      else
                //	 return ddx(ddxDerScheme, arr, dx, i);
                return ddx(ddxDerScheme, arr, dx, i);
            }

            public double[] ddx(double[] arr, double dx)
            {
                if      (ddxDerScheme == DDXDerScheme.ddx_fore)
                    return ddx_fore(arr,dx);
                else if (ddxDerScheme == DDXDerScheme.ddx_fore2)
                    return ddx_fore2(arr,dx);
                else if (ddxDerScheme == DDXDerScheme.ddx_back)
                    return ddx_back(arr,dx);
                else if (ddxDerScheme == DDXDerScheme.ddx_back2)
                    return ddx_back2(arr,dx);
                else if (ddxDerScheme == DDXDerScheme.ddx_foreMix4)
                    return ddx_foreMix4(arr,dx);
                else if (ddxDerScheme == DDXDerScheme.ddx_backMix4)
                    return ddx_backMix4(arr,dx);
                else if (ddxDerScheme == DDXDerScheme.ddx_cent5)
                    return ddx_cent5(arr,dx);
                else  if (ddxDerScheme == DDXDerScheme.ddx_cent)
                    return ddx_cent(arr,dx);
                else
                    return null;
            }

            public static double[] ddx_fore(double[] arr, double dx)
            {
                // 1st order foreward diff
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N-1; n++)
                    res[n] = ddx_fore(arr, dx, n);

                res[N-1] = ddx_back2(arr,dx,N-1);

                return res;      
            }

            public static double[] ddx_fore2(double[] arr, double dx)
            {
                // 2nd order foreward looking
                int N = arr.length;
                double[] res = new double[N];
                for (int n=0; n<N-2; n++)
                    res[n] = ddx_fore2(arr, dx, n);

                res[N-2] = ddx_back3(arr,dx,N-2);	// or use central??
                res[N-1] = ddx_back3(arr,dx,N-1);

                return res;      
            }

            public static double[] ddx_back(double[] arr, double dx)
            {
                // 1st order backward diff
                int N = arr.length;
                double[] res = new double[N];
                for (int n=1; n<N; n++)
                    res[n] = ddx_back(arr,dx,n);

                res[0] = ddx_fore2(arr,dx,0);

                return res;      
            }

            public static double[] ddx_back2(double[] arr, double dx)
            {
                // 2nd order backward diff
                int N = arr.length;
                double[] res = new double[N];
                for (int n=2; n<N; n++)
                    res[n] = ddx_back2(arr, dx, n);

                res[0] = ddx_fore3(arr,dx,0);
                res[1] = ddx_fore3(arr,dx,1);	// or use cent
                //res[1] = ddx_foreMix4(arr,dx,1);

                return res;      
            }

            public static double[] ddx_foreMix4(double[] arr, double dx)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=1; n<N-2; n++)
                    res[n] = ddx_foreMix4(arr,dx,n);

                res[0]   = ddx_fore5(arr,dx,0);
                res[N-2] = ddx_backMix4(arr,dx,N-2);
                res[N-1] = ddx_back5(arr,dx,N-1);

                return res;      
            }

            public static double[] ddx_backMix4(double[] arr, double dx)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=2; n<N-1; n++)
                    res[n] = ddx_backMix4(arr,dx,n);

                res[0]   = ddx_fore5(arr,dx,0);
                res[1]   = ddx_foreMix4(arr,dx,1);
                res[N-1] = ddx_back5(arr,dx,N-1);

                return res;      
            }

            public static double[] ddx_cent(double[] arr, double dx)
            {
                // 2nd order central differencing
                int N = arr.length;
                double[] res = new double[N];
                for (int n=1; n<N-1; n++)
                    res[n] = ddx_cent2(arr,dx,n);

                res[0]   = ddx_fore3(arr,dx,0);
                res[N-1] = ddx_back3(arr,dx,N-1);

                return res;      
            }

            public static double[] ddx_cent5(double[] arr, double dx)
            {
                int N = arr.length;
                double[] res = new double[N];
                for (int n=2; n<N-2; n++)
                    res[n] = ddx_cent5(arr,dx,n);

                res[0]   = ddx_fore3(arr,dx,0);
                res[1]   = ddx_fore3(arr,dx,1);
                res[N-2] = ddx_back3(arr,dx,N-2);
                res[N-1] = ddx_back3(arr,dx,N-1);

                return res;      
            }

            public static double ddx_cent2(double[] f, double dx, int i)
            {
                return (1.0/(2.0*dx))*(-1.0*f[i-1] + 1.0*f[i+1]);
            }
            public static double ddx_cent5(double[] f, double dx, int i)
            {
                return (1.0/(12.0*dx))*( 1.0*f[i-2] - 8.0*f[i-1] + 8.0*f[i+1] - 1.0*f[i+2]);
            }
            public static double ddx_fore(double[] f, double dx, int i)
            {
                return (1.0/dx)*(-1.0*f[i] + 1.0*f[i+1]);
            }
            public static double ddx_back(double[] f, double dx, int i)
            {
                return (1.0/dx)*( 1.0*f[i] - 1.0*f[i-1]);
            }
            public static double ddx_fore2(double[] f, double dx, int i)
            {
                return (1.0/(2.0*dx))*(-3.0*f[i] + 4.0*f[i+1] - 1.0*f[i+2]);
            }
            public static double ddx_back2(double[] f, double dx, int i)
            {
                return (1.0/(2.0*dx))*( 3.0*f[i] - 4.0*f[i-1] + 1.0*f[i-2]);
            }
            public static double ddx_fore3(double[] f, double dx, int i)
            {
                return (1.0/(6.0*dx))*(-11.0*f[i] + 18.0*f[i+1] - 9.0*f[i+2] + 2.0*f[i+3]);
            }
            public static double ddx_back3(double[] f, double dx, int i)
            {
                return (1.0/(6.0*dx))*( 11.0*f[i] - 18.0*f[i-1] + 9.0*f[i-2] - 2.0*f[i-3]);
            }
            public static double ddx_fore4(double[] f, double dx, int i)
            {
                return (1.0/(12.0*dx))*(-25.0*f[i] + 48.0*f[i+1] - 36.0*f[i+2] + 16.0*f[i+3] - 3.0*f[i+4]);
            }
            public static double ddx_back4(double[] f, double dx, int i)
            {
                return (1.0/(12.0*dx))*( 25.0*f[i] - 48.0*f[i-1] + 36.0*f[i-2] - 16.0*f[i-3] + 3.0*f[i-4]);
            }
            public static double ddx_fore5(double[] f, double dx, int i)
            {
                return (1.0/(60.0*dx))*(-137.0*f[i] + 300.0*f[i+1] - 300.0*f[i+2] + 200.0*f[i+3] - 75.0*f[i+4] + 12.0*f[i+5]);
            }
            public static double ddx_back5(double[] f, double dx, int i)
            {
                return (1.0/(60.0*dx))*( 137.0*f[i] - 300.0*f[i-1] + 300.0*f[i-2] - 200.0*f[i-3] + 75.0*f[i-4] - 12.0*f[i-5]);
            }
            public static double ddx_foreMix4(double[] f, double dx, int i)
            {
                return (1.0/(6.0*dx))*(-2.0*f[i-1] - 3.0*f[i] + 6.0*f[i+1] - 1.0*f[i+2]);
            }
            public static double ddx_backMix4(double[] f, double dx, int i)
            {
                return (1.0/(6.0*dx))*( 2.0*f[i+1] + 3.0*f[i] - 6.0*f[i-1] + 1.0*f[i-2]);
            }

            // --- d/dt functions --- //
            /*//   
   public double ddt(double[] arr, double[] arr0, double[] arr00, double dt, int i)
   {
      if      (ddtDerScheme == DDTDerScheme.ddt_back)
	 return ddt_back1(arr, arr0, dt, i);
      else if (ddtDerScheme == DDTDerScheme.ddt_back2)
	 return ddt_back2(arr, arr0, arr00, dt, i);
      else
	 return ddt_cent(arr, arr00, dt, i);
   }

   public double[] ddt(double[] arr, double[] arr0, double[] arr00, double dt)
   {

      if      (ddtDerScheme == DDTDerScheme.ddt_back)
	 return ddt_back1(arr, arr0, dt);
      else if (ddtDerScheme == DDTDerScheme.ddt_back2)
	 return ddt_back2(arr, arr0, arr00, dt);
      else
	 return ddt_cent(arr, arr00, dt);		// probably the most stable choice, though less acurate...

   }
//*/

            //

            public double ddt(double[] arr, double[] arr0, double[] arr00, double[] arr000, double dt, int i)
            {
                if      (ddtDerScheme == DDTDerScheme.ddt_back)
                    return ddt_back1(arr, arr0, dt, i);
                else if (ddtDerScheme == DDTDerScheme.ddt_back2)
                    return ddt_back2(arr, arr0, arr00, dt, i);
                else if (ddtDerScheme == DDTDerScheme.ddt_backMix4)
                    return ddt_backMix4(arr, arr0, arr00, arr000, dt, i);
                else
                    return ddt_cent(arr, arr00, dt, i);
            }

            public double[] ddt(double[] arr, double[] arr0, double[] arr00, double[] arr000, double dt)
            {

                if      (ddtDerScheme == DDTDerScheme.ddt_back)
                    return ddt_back1(arr, arr0, dt);
                else if (ddtDerScheme == DDTDerScheme.ddt_back2)
                    return ddt_back2(arr, arr0, arr00, dt);
                else if (ddtDerScheme == DDTDerScheme.ddt_backMix4)
                    return ddt_backMix4(arr, arr0, arr00, arr000, dt);
                else
                    return ddt_cent(arr, arr00, dt);		// probably the most stable choice, though less acurate...

            }

            //-----
            public static double ddt_back1(double[] arr1, double[] arr0, double dt, int n)
            {
                return (arr1[n] - arr0[n])/dt;	// first order backwards deriv
            }

            public static double ddt_back2(double[] arr2, double[] arr1, double[] arr0, double dt, int n)
            {
                return (3.0*arr2[n] -4.0*arr1[n] + 1.0*arr0[n])/(2.0*dt);
            }

            public static double ddt_back3(double[] arr3, double[] arr2, double[] arr1, double[] arr0, double dt, int n)
            {
                return (arr3[n] + arr2[n] - arr1[n] - arr0[n])/(4.0*dt);
            }

            public static double ddt_cent(double[] arr2, double[] arr0, double dt, int n)
            {
                return (1.0*arr2[n] - 1.0*arr0[n])/(2.0*dt);
            }

            public static double ddt_backMix4(double[] arr, double[] arr0, double[] arr00, double[] arr000, double dt, int n)
            {
                return (1.0/(6.0*dt))*( 2.0*arr[n] + 3.0*arr0[n] - 6.0*arr00[n] + 1.0*arr000[n]);
            }
            //-----

            public static double[] ddt_back1(double[] arr1, double[] arr0, double dt)
            {
                // 1st order time diff
                if (arr0.length == arr1.length)
                {
                    int N = arr0.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = (arr1[n] - arr0[n])/dt;

                    return res;
                }
                else
                {
                    return null;
                }
            }

            public static double[] ddt_back2(double[] arr2, double[] arr1, double[] arr0, double dt)
            {
                // calculates the 2nd order accurate backward derivative across the arrays (so the derivative is at time "2")
                if ( (arr0.length == arr1.length) && (arr0.length == arr2.length) )
                {
                    int N = arr0.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = (3.0*arr2[n] -4.0*arr1[n] + 1.0*arr0[n])/(2.0*dt);

                    return res;
                }
                else
                {
                    return null;
                }
            }

            public static double[] ddt_back3(double[] arr3, double[] arr2, double[] arr1, double[] arr0, double dt)
            {
                // calculates the 2nd order accurate backward derivative across the arrays (so the derivative is at time "2")
                if ( (arr0.length == arr1.length) && (arr0.length == arr2.length) && (arr0.length == arr3.length) )
                {
                    int N = arr0.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = (arr3[n] + arr2[n] - arr1[n] - arr0[n])/(4.0*dt);

                    return res;
                }
                else
                {
                    return null;
                }
            }

            public static double[] ddt_cent(double[] arr2, double[] arr0, double dt)
            {
                // calculates the 2nd order accurate central derivative across the arrays (so the derivative is at time "1")
                // warning: using this for time advance means that the ddt terms will actually be 1 time step behind the ddx terms
                if (arr0.length == arr2.length)
                {
                    int N = arr0.length;
                    double[] res = new double[N];
                    for (int n=0; n<N; n++)
                        res[n] = (1.0*arr2[n] - 1.0*arr0[n])/(2.0*dt);

                    return res;
                }
                else
                {
                    return null;
                }
            }

            public static double[] ddt_backMix4(double[] arr, double[] arr0, double[] arr00, double[] arr000, double dt)
            {
                // like "cent", this will find the deriv at n rather than n+1
                int N = arr0.length;
                double[] res = new double[N];
                for (int n=0; n<N; n++)
                    res[n] = ddt_backMix4(arr, arr0, arr00, arr000, dt, n);

                return res;
            }

            public static double min(double[] arr)
            {
                double min = arr[0];
                for (int a=1; a<arr.length; a++)
                {
                    if (arr[a] < min)
                        min = arr[a];
                }
                return min;
            }

            public static double max(double[] arr)
            {
                double max = arr[0];
                for (int a=1; a<arr.length; a++)
                {
                    if (arr[a] > max)
                        max = arr[a];
                }
                return max;
            }

            public static double sum(double[] arr)
            {
                double sum = 0.0;
                for (int a=0; a<arr.length; a++)
                {
                    sum = sum + arr[a];
                }
                return sum;
            }

            public static double average(double[] arr)
            {
                double N = (double)(arr.length);
                return sum(arr)/N;
            }

            public static double norm(double[] arr)
            {
                double norm = 0.0;
                for (int a=0; a<arr.length; a++)
                {
                    norm = norm + arr[a]*arr[a];
                }
                return Math.sqrt(norm);
            }

            public static double[] getArrayPart(double[] array, int i0, int i1)
            {
                int N = i1-i0+1;
                double[] part = new double[N];
                for (int i=0; i<N; i++)
                {
                    part[i] = array[i+i0];
                }
                return part;
            }


}
