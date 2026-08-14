package artisynth.models.fluid1d;

public class FluidSolver_1DDecoupled extends FluidSolver_1D_NSE
{
    // Solve the 1D, incompressible Navier Stokes in a decoupled solve: first solve u, then p
    // applicable BCs: u(inlet/outlet), p(inlet/outlet)...dp BCs are implemented by a hack...


    public void initializeSpecific()
    {      
        N = fsN.getNumberOfPoints();
        nVars = 1;		// well, 1 variable is solved at a time...
        nEqns = nVars*N;
    }

    //public FluidSolution_1D stepSpecific(double dt, FluidSolution_1D fsN, FluidSolution_1D fs, FluidSolution_1D fs0, FluidSolution_1D fs00)
    public void stepSpecific(double dt)
    {
        //if      ( (bcType == BCType.uIn_pIn) || (bcType == BCType.qIn_pIn) || (bcType == BCType.pIn_uOut) || (bcType == BCType.uIn_pOut) )
        if      ( (bcType == BCType.uIn_pIn) || (bcType == BCType.qIn_pIn) || (bcType == BCType.uIn_pOut)|| (bcType == BCType.uIn_pIn_dpLimited) || (bcType == BCType.uIn_pOut_dpLimited) )
        {
            stepInflow(dt);
        }
        else if (bcType == BCType.pIn_pOut)
        {
            stepPressure(dt);
        }
        else
        {
            // TODO: implement BCType uIn_pOut, pIn_uOut, resistor
            System.out.println("BC not implemented for decoupled solver; use coupled solver");
        }

    }

    public void stepInflow(double dt)
    {
        double rho    = fsN.getDensity();
        double mu     = fsN.getViscosity();
        double[] s    = fsN.getGeometry().getPerimeter();
        double[] A    = fsN.getGeometry().getArea();
        double[] u;	// not yet known
        double[] p;	// not yet known
        double[] tau;	// not yet known
        double[] A0   = fs.getGeometry().getArea();
        double[] u0   = fs.getVelocity();
        double[] p0   = fs.getPressure();
        //double[] tau0 = fs.getField("tau");
        double[] A00   = fs0.getGeometry().getArea();
        double[] u00   = fs0.getVelocity();
        //double[] p00   = fs0.getPressure();
        //double[] tau00 = fs0.getField("tau");
        double[] A000   = fs00.getGeometry().getArea();
        double[] u000   = fs00.getVelocity();

        // --- solve the mass equation for u [ ddx(A*u) = - ddt(A) ] --- //

        // q solve --> much better behavior!
        solver.clear();
        for (int n=0; n<N; n++)
        {
            defineSchemes(n,0);

            // --- Define the BCs --- //
            if      (equation == Equation.uInlet)
            {  // BC -->  u[0]*a[0] = uInlet*a[0]
                solver.addToB   (n,   uInlet*A[0]);	
                solver.scalarTerm(n,0, 1.0);
            }
            else if (equation == Equation.uOutlet)
            {  // BC -->  u[N-1]*a[N-1] = uOutlet*a[N-1]
                solver.addToB   (n,     uOutlet*A[N-1]);	
                solver.scalarTerm(n,N-1, 1.0);
            }
            else if (equation == Equation.qInlet)
            {  // BC -->  u[0]*a[0] = qInlet
                solver.addToB   (n,   qInlet);	
                solver.scalarTerm(n,0, 1.0);
            }
            else if (equation == Equation.qOutlet)
            {  // BC -->  u[N-1]*a[N-1] = qOutlet
                solver.addToB   (n,     qOutlet);	
                solver.scalarTerm(n,N-1, 1.0);
            }
            // --- Define the Fluid Equations --- //
            else if (equation == Equation.bernoulliMass)
            {
                // solve: u[i]*A[i] - u[i-1]*A[i-1] = 0
                solver.addToB(n,     0 );
                solver.scalarTerm(n,n,    1.0);
                solver.scalarTerm(n,n-1, -1.0);
            }
            else if (equation == Equation.nseMass)
            {  // solve: ddx(A*u) = - ddt(A)
                solver.addToB(n,     -1.0*cT*ddt(A, A0, A00, A000, dt, n)  );
                solver.ddxTerm(n,n,    1.0, ddxIntScheme, dx);
                if (fsN.fieldExists(gapName) == true)
                {
                    double gap = fsN.getField(gapName, n);
                    double pOut = 0.0;
                    double xAppx = 225.0;
                    //double qGap = Math.signum(p0[n]-pOut)*cGap*gap*dx*Math.sqrt((2.0/rho)*Math.abs(p0[n] - pOut));
                    double qGap = cGap*gap*dx*Math.sqrt(xAppx)*(Math.tanh( (2.0/rho)*(p0[n]-0.0)*(2.0/xAppx) ));
                    fsN.setField("q_"+gapName, n, qGap);
                    solver.addToB(n, -1.0*qGap/dx );
                }
            }
        }
        solver.solve();
        u = div(solver.getSolution(), A);	// I solved for A*u --> get u
        fsN.setField("u", u );

        // u solve
        //      solver.clear();
        //      for (int n=0; n<N; n++)
        //      {
        //	 defineSchemes(n,0);
        //	 
        //	 // --- Define the BCs --- //
        //	 if      (equation == Equation.uInlet)
        //	 {  // BC -->  u[0]*a[0] = uInlet*a[0]
        //	    solver.rhsTerm   (n,   uInlet);	
        //	    solver.scalarTerm(n,0, 1.0);
        //	 }
        //	 else if (equation == Equation.uOutlet)
        //	 {  // BC -->  u[N-1]*a[N-1] = uOutlet*a[N-1]
        //	    solver.rhsTerm   (n,     uOutlet);	
        //	    solver.scalarTerm(n,N-1, 1.0);
        //	 }
        //	 else if (equation == Equation.qInlet)
        //	 {  // BC -->  u[0]*a[0] = qInlet
        //	    solver.rhsTerm   (n,   qInlet/A[0]);	
        //	    solver.scalarTerm(n,0, 1.0);
        //	 }
        //	 else if (equation == Equation.qOutlet)
        //	 {  // BC -->  u[N-1]*a[N-1] = qOutlet
        //	    solver.rhsTerm   (n,     qOutlet/A[N-1]);	
        //	    solver.scalarTerm(n,N-1, 1.0);
        //	 }
        //	 
        //	 // --- Define the Fluid Equations --- //
        //       else if (equation == Equation.bernoulliMass)
        //	 {
        //	    // solve: u[i]*A[i] - u[i-1]*A[i-1] = 0
        //	    solver.rhsTerm(n,     0 );
        //	    solver.scalarTerm(n,n,    A[n]  );
        //	    solver.scalarTerm(n,n-1, -A[n-1]);
        //	 }
        //	 else if (equation == Equation.nseMass)
        //	 {  // solve: ddx(A*u) = A*ddx(u) + u*ddx(A) = - ddt(A)
        //	    solver.rhsTerm(n,     -1.0*cT*ddt(A, A0, A00, dt, n)  );
        //	    solver.ddxTerm(n,n,    A[n], ddxIntScheme, dx);
        //	    solver.scalarTerm(n,n, ddx(A,dx,n));
        //	 }
        //      }
        //      solver.solve();
        //      u = solver.getSolution();
        //      fsN.setField("u", u );

        // --- solve the momentum equation (for p) ---> -dp/dx = rho*du/dt + rho*u*du/dx - tau*s/A
        tau = calculateTau(dt, fsN, fs, fs0, fs00);		// calculate tau from u

        solver.clear();
        for (int n=0; n<N; n++)
        {
            defineSchemes(n,1);

            // --- Define the BCs --- //
            if      (equation == Equation.pInlet)
            {  // BC --> p[0] = pInlet
                solver.addToB   (n,    pInlet);
                solver.scalarTerm(n,0,  1.0);
            }
            else if (equation == Equation.pOutlet)
            {  // BC --> p[N-1] = pOutlet
                solver.addToB   (n,      pOutlet);
                solver.scalarTerm(n,N-1,  1.0);
            }
            // --- Define the Fluid Equations --- //
            else if (equation == Equation.bernoulliMom)
            {  
                solver.addToB(n,     - rho/2.0*(u[n]*u[n] - u[n-1]*u[n-1]) + tau[n]*(s[n]*dx/A[n]) );
                solver.scalarTerm(n,n,    1.0);
                solver.scalarTerm(n,n-1, -1.0);
            }
            else if (equation == Equation.nseMom)
            {
                // solve: -dp/dx = rho*du/dt + rho*u*du/dx - tau*s/A 
                solver.addToB(n,    cT*rho*ddt(u, u0, u00, u000, dt, n) + rho*u[n]*ddx(u, dx, n) - tau[n]*(s[n]/A[n]));
                solver.ddxTerm(n,n, -1.0, ddxIntScheme, dx);
            }
        }
        solver.solve();
        p = solver.getSolution();

        fsN.setField("p", p );
        fsN.setField("tau", tau);

    }

    public double[] calculateTau(double dt, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00, FluidSolution_1D X000)
    {
        // TODO: I think this section is identical to the tau calc in the coupled model, only the last line is different...combine?
        double rho    = X.getDensity();
        double mu     = X.getViscosity();
        double epsilon = 0.0;		// roughness
        double[] A    = X.getGeometry().getArea();
        double[] s    = X.getGeometry().getPerimeter();
        double[] u    = X.getVelocity();
        double[] u0   = X0.getVelocity();
        double[] u00  = X00.getVelocity();
        double[] u000 = X000.getVelocity();
        double[] chi  = calculateChi(dt, X, X0, X00, X000);
        double[] tau  = new double[N];

        for (int n=0; n<N; n++)
        {
            defineSchemes(n,2);

            double tauFriction = 0.0;
            double tauSmall = 0.0;
            double tauChi = 0.0;
            if (useFrictionLosses == true)
            {
                tauFriction = -2.0*mu*u[n]*(s[n]/A[n]);


                //	    double dHyd = 4.0*A[n]/s[n];	// hydraulic diameter
                //	    double Re = rho*u[n]*dHyd/mu;	// Reynolds number
                //	    double fLam  = 64.0/Re;
                //	    double fTurb = Math.pow(1.0/(-1.8*Math.log10(6.9/Re + Math.pow((epsilon/dHyd)/3.7, 1.11))), 2.0);
                //	    double f = weightFunc(Re, 2000.0, 4000.0, fLam, fTurb, 1.0, TransitionType.tanh);
                //	    tauFriction = -(1.0/2.0)*rho*u[n]*u[n]* (f/4.0);

                // below is the good version. TODO: I need to option this!
                //	    double dHyd = 4.0*A[n]/s[n];	// hydraulic diameter
                //	    double Re = Math.abs(rho*dHyd*u[n]/mu);							// no -#s allowed!
                //	    Re = FluidSolver_1D_NSE.weightFunc(Re, 0.0, 1.0, 0.64, Re, 1.0, TransitionType.tanh);	// no 0's allowed!
                //
                //	    double fLam  = 64.0/Re;
                //	    double fTurb = Math.pow(1.0/(-1.8*Math.log10(6.9/Re + Math.pow((epsilon/dHyd)/3.7, 1.11))), 2.0);
                //	    if ( (Double.isNaN(fTurb) == true) || (Double.isInfinite(fTurb) == true) || (fTurb > 500.0) )
                //	       fTurb = fLam;		// the discontinuity at Re=6.9 needs to be handled
                //	    double f = Math.signum(u[n]) * FluidSolver_1D_NSE.weightFunc(Re, 2000.0, 4000.0, fLam, fTurb, 3.0, TransitionType.sinusoid);
                //	    tauFriction = -(1.0/2.0)*rho*u[n]*u[n] * (f/4.0);
            }
            if (useTauSmall == true)
            {	    
                //tauSmall = weightFunc(A[n], areaSmallLimit, areaClosedLimit, 0.0, -rho*u[n]*u[n]/10.0, 5.0, TransitionType.tanh);
                tauSmall = weightFunc(A[n], areaSmallLimit, areaClosedLimit, 0.0, -rho*u[n]*u[n]/2.0, 5.0, TransitionType.linear);
                //tauSmall = weightFunc(A[n], areaSmallLimit, areaClosedLimit, 0.0, -inletPMax, 1.5, TransitionType.tanh);
            }
            if (useGeometryLosses == true)
            {
                // TODO: define this!
            }
            if (chiType != ChiTerm.None)
            {	    
                double tSpace = rho*u[n]*ddx(u,dx,n);
                double tTime  = rho*ddt(u, u0, u00, u000, dt,n)*cT;


                if      (chiType == ChiTerm.ChiSteady)
                    tauChi = (1.0-chi[n])*(A[n]/s[n])*tSpace;
                else if (chiType == ChiTerm.ChiUnsteady)
                    tauChi = (1.0-chi[n])*(A[n]/s[n])*(tSpace + tTime);
            }

            tau[n] = tauFriction + tauChi + tauSmall;
        }
        return tau;
    }

    public void stepPressure(double dt)
    {
        // TODO: this seems to fail with a negative pressure define...
        // iterate the inlet velocity until the proper outlet pressure (pOutlet) is found
        double tol = 0.05;			// convergence criteria: need not be very demanding in this case
        double frac = 0.60;			// if interpolating between bounds, this is percent towards the overshot boundary
        boolean defLow = false;
        boolean defHigh = false;
        double pLow=0.0, pHigh=0.0, uLow=0.0, uHigh=0.0;		// define bounds to ensure convergence
        double u1, u2, p1, p2;
        double diff = tol*10.0;
        double dpdu;

        double uInletOrig = uInlet;
        double pOutletOrig = pOutlet;
        BCType bcTypeOrig = bcType;

        bcType = BCType.uIn_pIn;

        // initial guess
        u2 = 1.0;
        if ( Math.abs(fsN.getVelocity(0)) > 0.000000001 )
            u2 = fsN.getVelocity(0);
        uInlet = u2;
        stepInflow(dt);
        p2 = fsN.getField("p", N-1);

        dpdu = (pOutlet-p2)/(u2*0.1);		// init dpdu such that the next uInlet = 1.1*u2

        // iterate
        int iters = 0;
        while ( Math.abs(diff) > tol )
        {
            u1 = u2;
            p1 = p2;

            // if (pLow < p1 < pOutlet) || (p1 < pOutlet and pLow isn't defined) --> pLow = p1
            if  ( p1 < pOutlet )
            {
                if ( (defLow == false) || (p1 > pLow) )
                {
                    pLow = p1;
                    uLow = u1;
                    defLow = true;
                }
            }
            else if (p1 > pOutlet)
            {
                if ( (defHigh == false) || (p1 < pHigh) )
                {
                    pHigh = p1;
                    uHigh = u1;
                    defHigh = true;
                }
            }

            u2 = (1.0/dpdu)*(pOutlet-p1) + u1;
            if ( (defLow == true) && (defHigh == true) )
            {
                if      ( (uLow > uHigh) && (u2 > uLow) )
                    u2 = uHigh + frac*(uLow - uHigh);
                else if ( (uLow > uHigh) && (u2 < uHigh) )
                    u2 = uLow  - frac*(uLow - uHigh);
                else if ( (uLow < uHigh) && (u2 < uLow) )
                    u2 = uHigh - frac*(uHigh - uLow);
                else if ( (uLow < uHigh) && (u2 > uHigh) )
                    u2 = uLow  + frac*(uHigh - uLow);
            }
            if (Math.abs(u2-u1) < 0.00000000001)
            {
                uInlet = (u2+u1)*0.5;
                stepInflow(dt);
                System.out.println("Warning: failed convergence in decoupled pressure iterations");
                break;
            }
            uInlet = u2;
            stepInflow(dt);
            p2 = fsN.getField("p", N-1);

            diff = pOutlet - p2;
            dpdu = (p2-p1)/(u2-u1);

            iters = iters + 1;
        }
        String sdl = "-";
        if (defLow == true)
            sdl = "L";
        String sdh = "-";
        if (defHigh == true)
            sdh = "H";

        uInlet  = uInletOrig;
        pOutlet = pOutletOrig;
        bcType  = bcTypeOrig;

        System.out.printf("Fluid Solver: pressure BCs. %d iterations. %s/%s \n", iters, sdl, sdh);
    }

    @Override
    public void setBCs_PressureExtended(double pInlet, double pOutlet, double lInlet, double lOutlet)
    {
        System.out.println("Decoupled Solver Warning: BC not implemented, switching to pressure BCs");
        bcType = BCType.pIn_pOut;
        this.pInlet = pInlet;
        this.pOutlet = pOutlet;
        this.lInlet = lInlet;
        this.lOutlet = lOutlet;
    }

}
