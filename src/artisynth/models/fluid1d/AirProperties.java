package artisynth.models.fluid1d;

public class AirProperties 
{
    public double T;	// temperature
    public double rho;			// density
    public double mu;			// viscosity
    public double nu;			// kinematic viscosity		nu = mu/rho
    public double R;			// gas constant			R = c_p - c_v
    public double pATM;			// atmospheric pressure		p = rho*R*T
    public double c;			// speed of sound		c = sqrt(gamma*p/rho)
    public double gamma;		// ratio of specific heats	gamma = c_p/c_v

    public AirProperties()
    {
        T = 20.0 + 273.15;		// [kelvin]
        mu = 0.000018;          		// [Pa*s = kg/(m*s)]
        rho = 1.2;			// [kg/m^3] 		appx value for 20C and dry (moist air is less dense)
        nu = mu/rho;			// [m^2/s] 		nu = 0.000015, or nu = 0.0000155
        R = 287.058;			// [J/(kg*K)]
        gamma = 1.4;			// [] 			for a perfect gas
        pATM = rho*R*T;			// [Pa]
        c = Math.sqrt(gamma*pATM/rho);	// [m/s]

        //pATMstd = 101325.0;	// Pa --> at T=0C
    }

}
