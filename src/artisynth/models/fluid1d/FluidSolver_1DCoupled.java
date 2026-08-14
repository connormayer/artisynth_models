package artisynth.models.fluid1d;

import artisynth.models.fluid1d.DESolver;

public class FluidSolver_1DCoupled extends FluidSolver_1D_NSE
{
    // Solve the 1D, incompressible Navier Stokes

    boolean useTransientJac = true;	// I can probably remove this option...
    double tol = 0.00000001;
    public int nSlack = 0;			// for some frustrating reason filling the Jac with diagonal 1s makes the simulation more stable...

    public void initializeSpecific()
    {
        N = fsN.getNumberOfPoints();
        nVars = 3 + nSlack;
        nEqns = nVars*N;
    }

    //public FluidSolution_1D stepSpecific(double dt, FluidSolution_1D fsN, FluidSolution_1D fs, FluidSolution_1D fs0, FluidSolution_1D fs00)
    public void stepSpecific(double dt)
    {
        // make an initial estimate
        estimateSolution( fsN, fs, fs0, fs00, fs000 );

        double[] F = new double[nEqns];
        calculateF(F, dt, fsN, fs, fs0, fs00);
        double residual = norm( F );
        int iter = 0;
        int maxIters = 50;

        while ( (residual > tol) && (iter < maxIters) )
        {
            //calculateF(F,      dt, fsN, fs, fs0);

            //System.out.printf("Clear solver... ");	// remove
            solver.clear();
            //solver = new DESolver_Sparse();
            //solver.setNumberOfPoints(nEqns);
            calculateJ(solver, dt, fsN, fs, fs0, fs00);

            solver.addToB(mult(-1.0,F));
            //System.out.printf("Solving J... ");		// remove
            //	 String tempWriteDir = "/home/peter/temp/matrixSpew/";
            //	 if (t>0.2)
            //	 {
            //	    solver.writeA( String.format("%smat_A_%f_%d", tempWriteDir, t, iter) );
            //	    //solver.writeb( String.format("/home/peter/temp/matrix_b_%f_%d.csv", t, iter) );
            //	 }


            /*/
	 // tinker with matrix damping...
	 //solver.addMatrixDamping_scaled(0.001);
	 //solver.scaleSystem();
	 for (int k=0; k<N; k++)
	 {
	    //solver.addToA(k, k, 0.000001);			// constant
	    //solver.addToA(k, k, 0.0000001*F[k]);			// scale by F
	    //solver.addToA(k,k, 0.00001*solver.getA(k,k) );	// scale by J
	    // adjust lambda if it benefits resid...
	 }
	 //*/

            solver.solve();
            double matrixR = solver.computeResidual();
            //double matrixR = -1.0;
            //System.out.printf("pRes=%e,  ", res);
            //	 if ( res > 0.0001)
            //	 {
            //	    solver.writeA( String.format("%smatA_%f_%d", tempWriteDir, t, iter) );
            //	    solver.writeb( String.format("%smatB_%f_%d", tempWriteDir, t, iter) );
            //	 }
            double[] dX = solver.getSolution();

            double j = 0.0;
            double residualNew = residual*10.0;		// initialize to be sure we enter the line-search loop
            double[] X0 = getVectorFromState(fsN);
            //System.out.printf("Weight F... ");
            while (residual < residualNew)
            {
                double[] X1 = add(X0, mult(Math.pow(0.5, j), dX) );
                setVectorInState(fsN, X1);
                calculateF(F, dt, fsN, fs, fs0, fs00);
                residualNew = norm( F );
                j = j+1.0;
                if (j>30.0)
                    break;
            }
            residual = residualNew;
            iter++;
            if (verbose==true)
                System.out.printf("Iter = %d,  linesearchIter = %04.1f,  eqnR = %f, matR = %f,  DXnorm = %f \n", iter, j, residual, matrixR, norm(dX));
        }
        System.out.println("Step Completed");

    }

    public void calculateF(double[] F, double dt, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00, FluidSolution_1D X000)
    {
        /*//
         * F1 = cons mass (bernoulli or NSE)
         * F2 = cons mom  (bernoulli or NSE)
         * F3 = viscous
         * F4 = velocity at n   (for ddt terms)
         * F5 = velocity at n-1 (for ddt terms)
      //*/

        double rho    = X.getDensity();
        double mu     = X.getViscosity();
        double[] s    = X.getGeometry().getPerimeter();
        double[] A    = X.getGeometry().getArea();
        double[] u    = X.getVelocity();
        double[] p    = X.getPressure();
        double[] tau  = X.getField("tau");
        double[] A0   = X0.getGeometry().getArea();
        double[] u0   = X0.getVelocity();
        //double[] p0   = X0.getPressure();
        //double[] tau0 = X0.getField("tau");
        double[] A00   = X00.getGeometry().getArea();
        double[] u00   = X00.getVelocity();
        //double[] p00   = X00.getPressure();
        //double[] tau00 = X00.getField("tau");
        double[] A000   = X000.getGeometry().getArea();
        double[] u000   = X000.getVelocity();
        //double[] p000   = X000.getPressure();
        //double[] tau000 = X000.getField("tau");
        double[] chi = calculateChi(dt, X, X0, X00, X000);


        double[] Au = mult(A,u);

        for (int b=0; b<3; b++)
        {
            for (int n=0; n<N; n++)
            {
                defineSchemes(n,b);

                switch (equation)
                {
                    // --- fluid equations --- /
                    case bernoulliMass:
                        F[b*N+n] = u[n]*A[n] - u[n-1]*A[n-1];
                        break;
                    case bernoulliMom:
                        F[b*N+n] = p[n] - p[n-1] + rho/2.0*(u[n]*u[n] - u[n-1]*u[n-1]) - tau[n]*(s[n]*dx/A[n]);
                        break;

                    case nseMass:
                        double qGap = 0.0;
                        if (fsN.fieldExists(gapName) == true)
                        {
                            double xAppx = 225.0;
                            double gap = fsN.getField(gapName, n);
                            //qGap = Math.signum(p[n]-0.0)*cGap*gap*dx*Math.sqrt( (2.0/rho)*Math.abs(p[n]-0.0) );
                            qGap = cGap*gap*dx*Math.sqrt(xAppx)*(Math.tanh( (2.0/rho)*(p[n]-0.0)*(2.0/xAppx) ));
                            fsN.setField("q_"+gapName, n, qGap);
                        }
                        //F1 = ddt(A)*cT + ddx(A*u) = ddt(A)*cT + A*ddx(u) + u*ddx(A) = 0;
                        //F[0*N+n] = ddt(A, A0, A00, dt, n)*cT + A[n]*ddx(u,dx,n) + u[n]*ddx( A, dx, n);
                        F[b*N+n] = ddt(A, A0, A00, A000, dt, n)*cT + ddx(Au,dx,n) + qGap/dx;		// It is more stable to solve ddx(A*u) rather than A*ddx(u) + u*ddx(A)...

                        break;
                    case nseMom:
                        //F2 = rho*ddt(u)*cT + rho*u*ddx(u) + ddx(p) - tau*s/A = 0;
                        F[b*N+n] = rho*ddt(u, u0, u00, u000, dt,n)*cT + rho*u[n]*ddx(u,dx,n) + ddx(p,dx,n) - tau[n]*(s[n]/A[n]);
                        break;

                        // --- viscous equation --- //
                    case tau:
                        double tauFriction = 0.0;
                        double tauSmall = 0.0;
                        double tauChi = 0.0;
                        if (useFrictionLosses == true)
                        {
                            tauFriction = -2.0*mu*u[n]*(s[n]/A[n]);
                            // TODO: include the laminar and turbulent friction options
                        }
                        if (useGeometryLosses == true)
                        {
                            // TODO: define this!
                        }
                        if (useTauSmall == true)
                        {
                            //tauSmall = -rho*u[n]*u[n]*1.0*Math.exp(-1.0*(A[n]-areaClosedLimit)/(areaSmallLimit-areaClosedLimit));

                            //tauSmall = -rho*u[n]*u[n] * weightFunc(A[n], areaSmallLimit, areaClosedLimit);
                            tauSmall = -inletPMax * weightFunc(A[n], areaSmallLimit, areaClosedLimit);
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

                        F[b*N+n] = tau[n] - tauFriction - tauChi - tauSmall;
                        break;

                        // --- closed equations --- //
                    case closed_u:
                        closedFunction(F,n,b,X, u);
                        break;
                    case closed_p:
                        closedFunction(F,n,b,X, p);
                        break;
                    case closed_tau:
                        closedFunction(F,n,b,X, tau);
                        break;

                        // --- boundary conditions --- //
                    case uInlet:
                        F[b*N+n] = u[0] - uInlet;
                        break;
                    case uOutlet:
                        F[b*N+n] = u[N-1] - uOutlet;
                        break;
                    case qInlet:
                        F[b*N+n] = u[0] - qInlet/A[0];
                        break;
                    case qOutlet:
                        F[b*N+n] = u[N-1] - qOutlet/A[N-1];
                        break;
                    case pInlet:
                        F[b*N+n] = p[0] - pInlet;
                        break;
                    case pOutlet:
                        F[b*N+n] = p[N-1] - pOutlet;
                        break;
                    case pInletExt:
                        F[b*N+n] =  p[0]   - pInlet - lInlet*(s[0]/A[0])*tau[0] +rho*lInlet*ddt(u,u0,u00, u000, dt, 0)*cT;
                        break;
                    case pOutletExt:
                        F[b*N+n] = -p[N-1] + pOutlet - lOutlet*(s[N-1]/A[N-1])*tau[N-1] +rho*lOutlet*ddt(u,u0,u00, u000, dt, N-1)*cT;
                        break;
                    case rInlet:
                        F[b*N+n] = pInlet - p[0]    - rInlet *u[0]   - iInlet *ddt(u,u0,u00, u000, dt, 0)*cT;
                        break;
                    case rOutlet:
                        F[b*N+n] = p[N-1] - pOutlet - rOutlet*u[N-1] - iOutlet*ddt(u,u0,u00, u000, dt, N-1)*cT;
                        break;
                    case tauBC:
                        F[b*N+n] = ddx(tau, dx, n) - 0.0;
                        break;

                    case dummy:
                        break;
                    default:
                        break;

                }
            }
        }

        // define the slack equations...
        //      for (int n=0; n<N; n++)
        //      {
        //	 F[3*N+n] = 0.0; 
        //	 F[4*N+n] = 0.0;
        //	 F[5*N+n] = 0.0;
        //      }

    }

    public void calculateJ(DESolver S, double dt, FluidSolution_1D X, FluidSolution_1D X0, FluidSolution_1D X00, FluidSolution_1D X000)
    {
        double rho    = X.getDensity();
        double mu     = X.getViscosity();
        double[] s    = X.getGeometry().getPerimeter();
        double[] A    = X.getGeometry().getArea();
        double[] u    = X.getVelocity();
        double[] p    = X.getPressure();
        double[] tau  = X.getField("tau");
        //      double[] A0   = X0.getGeometry().getArea();
        //      double[] u0   = X0.getVelocity();
        //      double[] p0   = X0.getPressure();
        //      double[] tau0 = X0.getField("tau");
        //      double[] A00   = X00.getGeometry().getArea();
        //      double[] u00   = X00.getVelocity();
        //      double[] p00   = X00.getPressure();
        //      double[] tau00 = X00.getField("tau");
        double[] chi = calculateChi(dt, X, X0, X00, X000);

        int iu0 = 0;
        int iuN = N-1;
        int ip0 = N;
        int ipN = 2*N-1;
        int it0 = 2*N;
        int itN = 3*N-1;

        for (int b=0; b<3; b++)
        {
            for (int n=0; n<N; n++)
            {
                defineSchemes(n,b);

                switch (equation)
                {
                    // --- fluid equations --- /
                    case bernoulliMass:
                        // --- define dF1/du
                        S.scalarTerm(b*N+n,iu0+n,    A[n]);
                        S.scalarTerm(b*N+n,iu0+n-1, -A[n-1]);

                        // --- define dF1/dp --> everywhere 0

                        // --- define dF1/dtau --> everywhere 0
                        break;
                    case bernoulliMom:
                        // --- define dF2/du
                        S.scalarTerm(b*N+n,iu0+n,    rho*u[n]);
                        S.scalarTerm(b*N+n,iu0+n-1, -rho*u[n-1]);

                        // --- define dF2/dp
                        S.scalarTerm(b*N+n,ip0+n,    1.0);
                        S.scalarTerm(b*N+n,ip0+n-1, -1.0);

                        // --- define dF2/dtau
                        S.scalarTerm(b*N+n,it0+n, -(s[n]*dx/A[n]));
                        break;

                    case nseMass:
                        // --- F1 --- // 
                        // dF1/du
                        S.scalarTerm(b*N+n,iu0+n, ddx(A, dx, n));
                        S.ddxTerm   (b*N+n,iu0+n, A[n], ddxIntScheme, dx);

                        // define dF1/dp --> everywhere 0
                        if (fsN.fieldExists(gapName) == true)
                        {
                            double gap = fsN.getField(gapName, n);
                            //double gapTerm = cGap*gap*dx/Math.sqrt( 2.0*rho*Math.abs(p[n]-0.0) );	// I can just drop dx...
                            double xAppx = 225.0;
                            double gapTerm = cGap*gap*dx*4.0/(rho*Math.sqrt(xAppx)) * (1.0 - Math.pow(Math.tanh( (2.0/rho)*(p[n]-0.0)*(2.0/xAppx) ), 2.0));

                            S.scalarTerm(b*N+n,ip0+n, gapTerm/dx);
                        }

                        // define dF1/dtau --> everywhere 0
                        break;
                    case nseMom:
                        //  --- F2 --- //
                        // define dF2/du
                        S.scalarTerm(b*N+n,iu0+n, rho*ddx(u, dx, n));
                        S.ddxTerm   (b*N+n,iu0+n, rho*u[n], ddxIntScheme, dx);
                        //S.ddtTerm   (b*N+n,iu0+n, 3*N+n,4*N+n,5*N+n, rho*cT, ddtIntScheme, dt);
                        S.ddtTermUsingRHS(b*N+n,iu0+n, rho*cT, ddtIntScheme, dt, 0.0, 0.0);	// my solution at u0 is *not* changing --> du0 = u0_n+1 - u0_n = 0

                        //  define dF2/dp
                        S.ddxTerm   (b*N+n,ip0+n, 1, ddxIntScheme, dx);

                        // define dF2/dtau
                        S.scalarTerm(b*N+n,it0+n, -s[n]/A[n]);
                        //S.scalarTerm(N+n,2*N+n, (-1.0/A[n])*dadx[n]);
                        //S.ddxTerm(N+n,2*N+n, -1.0, ddxScheme, dx);
                        break;

                        // --- viscous equation --- //
                    case tau:
                        // dF3/dU   = -dTauFric/dU - dTauChi/dU
                        // dF3/dP   = -dTauFric/dP - dTauChi/dP
                        // dF3/dTau = 1 - dTauFric/dTau - dTauChi/dTau = 1
                        S.scalarTerm(b*N+n,it0+n, 1.0);

                        if (useFrictionLosses == true)
                        {
                            // dTauFric/dU	--> -2.0*mu*(s/A)
                            double dTAUFRICdu_S = -2.0*mu*s[n]/A[n];
                            S.scalarTerm(b*N+n,iu0+n,              -dTAUFRICdu_S );

                            // dTauFric/dP	--> zero
                            // dTauFric/dTau	--> zero
                        }
                        if (useGeometryLosses == true)
                        {	// undefined...
                        }
                        if (useTauSmall == true)
                        {
                            // dTauSmall/dU	= -2.0*rho*u*exp(-10.0*(A-Aclosed)/(Asmall-Aclosed))
                            //double dTAUSMALLdu_S = -2.0*rho*u[n]*1.0*Math.exp(-1.0*(A[n]-areaClosedLimit)/(areaSmallLimit-areaClosedLimit));
                            double dTAUSMALLdu_S = -2.0*rho*u[n] * weightFunc(A[n], areaSmallLimit, areaClosedLimit);
                            //S.scalarTerm(b*N+n,iu0+n,              -dTAUSMALLdu_S );

                            // dTauFric/dP	--> zero
                            // dTauFric/dTau	--> zero
                        }
                        if (chiType != ChiTerm.None)
                        {
                            if      ( (chiType == ChiTerm.ChiSteady) || (chiType == ChiTerm.ChiUnsteady) )	// hack...
                            {
                                //		     // dTauChi/dU		--> messy! 
                                //		     double dudt = ddt(u,u0,u00,dt,n)*cT;
                                //		     double dudx = ddx(u,dx,n);
                                //		     double Psi = (-sfScalar*rho*(chiMax-chiMin)/Math.PI) / (1.0 + Math.pow((rho*sfScalar*u[n])*(dudt + u[n]*ddx(u,dx,n)),2.0) );
                                //
                                //		     double dTAUCHIdu_S  = rho*(A[n]/s[n]) * (Psi*u[n]*dudx*(dudt + 2.0*u[n]*dudx) + (1.0 - chi)*dudx);
                                //		     double dTAUCHIdu_dt = rho*(A[n]/s[n]) * (Psi*u[n]*u[n]*dudx);
                                //		     double dTAUCHIdu_dx = rho*(A[n]/s[n]) * (Psi*Math.pow(u[n],3.0) * dudx + (1.0-chi)*u[n]);
                                //
                                //		     S.scalarTerm(b*N+n,iu0+n,              -dTAUCHIdu_S);
                                //		     S.ddxTerm   (b*N+n,iu0+n,              -dTAUCHIdu_dx,    ddxIntScheme, dx);
                                //		     S.ddtTerm   (b*N+n,iu0+n, 3*N+n,4*N+n, -dTAUCHIdu_dt*cT, ddtIntScheme, dt);

                                double dTAUCHIdu_S  = rho*(A[n]/s[n]) * (-u[n]*ddx(chi, dx, n) + (1.0 - chi[n])*ddx(u,dx,n));
                                double dTAUCHIdu_dx = rho*(A[n]/s[n]) * ( (1.0-chi[n])*u[n]);

                                S.scalarTerm(b*N+n,iu0+n,              -dTAUCHIdu_S);
                                S.ddxTerm   (b*N+n,iu0+n,              -dTAUCHIdu_dx,    ddxIntScheme, dx);
                                // dTauChi/dP   = zero
                                // dTauChi/dTau = zero
                            }
                            else if (chiType == ChiTerm.ChiUnsteady)
                            {
                                // not defined...just using the static no-gain terms
                            }
                        }
                        break;

                        // --- closed equations --- //
                    case closed_u:
                        closedFunctionJac(S,n,b,X, u);
                        break;
                    case closed_p:
                        closedFunctionJac(S,n,b,X, p);
                        break;
                    case closed_tau:
                        closedFunctionJac(S,n,b,X, tau);
                        break;

                        // --- boundary conditions --- //
                    case uInlet:
                        S.scalarTerm(b*N+n, iu0, 1.0);
                        break;
                    case uOutlet:
                        S.scalarTerm(b*N+n, iuN, 1.0);
                        break;
                    case qInlet:
                        S.scalarTerm(b*N+n, iu0, 1.0);
                        break;
                    case qOutlet:
                        S.scalarTerm(b*N+n, iuN, 1.0);
                        break;
                    case pInlet:
                        S.scalarTerm(b*N+n, ip0, 1.0);
                        break;
                    case pOutlet:
                        S.scalarTerm(b*N+n, ipN, 1.0);
                        break;

                    case pInletExt:
                        //F[b*N+n] =  p[0]   - pInlet - lInlet*(s[0]/A[0])*tau[0] +rho*lInlet*ddt(u,u0,u00, u000, dt, 0)*cT;
                        //S.ddtTerm   (b*N+n, iu0, 3*N+iu0,4*N+iu0,5*N+iu0,  rho*lInlet*cT, ddtIntScheme, dt);
                        S.ddtTermUsingRHS(b*N+n, iu0, rho*lInlet*cT, ddtIntScheme, dt, 0.0, 0.0);
                        S.scalarTerm(b*N+n, ip0, 1.0);
                        S.scalarTerm(b*N+n, it0, -lInlet*(s[n]/A[n]) );
                        break;
                    case pOutletExt:
                        //F[b*N+n] = -p[N-1] + pOutlet - lOutlet*(s[N-1]/A[N-1])*tau[N-1] +rho*lOutlet*ddt(u,u0,u00, u000, dt, N-1)*cT;
                        //S.ddtTerm        (b*N+n, iuN, 3*N+iuN,4*N+iuN,5*N+iuN,  rho*lOutlet*cT, ddtIntScheme, dt);
                        S.ddtTermUsingRHS(b*N+n, iuN, rho*lOutlet*cT, ddtIntScheme, dt, 0.0, 0.0);
                        S.scalarTerm(b*N+n, ipN, -1.0);
                        S.scalarTerm(b*N+n, itN, -lOutlet*(s[n]/A[n]) );
                        break;

                    case rInlet:
                        S.scalarTerm(b*N+n, iu0,                   -rInlet);
                        //S.ddtTerm        (b*N+n, iu0, 3*N+iu0,4*N+iu0,5*N+iu0,  -iInlet*cT, ddtIntScheme, dt);
                        S.ddtTermUsingRHS(b*N+n, iu0, -iInlet*cT, ddtIntScheme, dt, 0.0, 0.0);
                        S.scalarTerm(b*N+n, ip0, -1.0);
                        break;
                    case rOutlet:
                        S.scalarTerm(b*N+n, iuN,                   -rOutlet);
                        //S.ddtTerm        (b*N+n, iu0,3*N+iu0,4*N+iu0,5*N+iu0,   -iOutlet*cT, ddtIntScheme, dt);	// TODO: shouldn't this be iuN
                        S.ddtTermUsingRHS(b*N+n, iuN, -iOutlet*cT, ddtIntScheme, dt, 0.0, 0.0);
                        S.scalarTerm(b*N+n, ipN,  1.0);
                        break;
                    case tauBC:
                        S.ddxTerm(b*N+n, it0, 1.0, ddxIntScheme, dx);
                        break;

                    case dummy:
                        break;
                    default:
                        break;

                }
            }
        }

        // define the slack equations...
        for (int m=0; m<nSlack; m++)
        {
            for (int n=0; n<N; n++)
            {
                S.scalarTerm((3+m)*N+n, (3+m)*N+n,  1.0);
            }
        }


    }

    public void setVectorInState(FluidSolution_1D x, double[] v)
    {
        for (int n=0; n<N; n++)
        {
            x.setVelocity(    n, v[0*N + n]);
            x.setPressure(    n, v[1*N + n]);
            x.setField("tau", n, v[2*N + n]);
        }
    }

    public double[] getVectorFromState(FluidSolution_1D x)
    {
        double[] v = new double[nEqns];

        for (int n=0; n<N; n++)
        {
            v[0*N+n] = x.getVelocity(n);
            v[1*N+n] = x.getPressure(n);
            v[2*N+n] = x.getField("tau", n);
        }

        return v;
    }

}
