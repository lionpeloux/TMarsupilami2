using System;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using TMarsupilami.CoreLib3;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum SolverState : int
    {
        Running,
        Converged,
        Ended,
        Aborted
    }

    public class KDRSolver
    {
        public bool IsParallelModeEnabled { get; private set; }
        public ParallelOptions ParallelOptions { get; private set; }

        public SolverState State { get; protected set; }
        public int MaxIteration { get; protected set; }

        public int CurrentIteration_x { get; protected set; }
        public int CurrentIteration_θ { get; protected set; }

        public int NumberOfKineticPeaks_x { get; protected set; }
        public int NumberOfKineticPeaks_θ { get; protected set; }

        // SOLVER STATE EVENTS
        public event Action<KDRSolver> OnEnergyPeak_x;
        public event Action<KDRSolver> OnEnergyPeak_θ;
        public event Action<KDRSolver> OnConvergence;
        public event Action<KDRSolver> OnNotConvergence;

        // EVENTS
        private ParallelActionHandler InitElementsHandler;
        protected void OnInitElements(bool parallel = false)
        {
            InitElementsHandler.Raise(parallel);
        }
        public event Action InitElements
        {
            add { InitElementsHandler.Subscribe(value); }
            remove { InitElementsHandler.UnSubscribe(value); }
        }

        private ParallelActionHandler InitConstraintsHandler;
        protected void OnInitConstraints(bool parallel = false)
        {
            InitConstraintsHandler.Raise(parallel);
        }
        public event Action InitConstraints
        {
            add { InitConstraintsHandler.Subscribe(value); }
            remove { InitConstraintsHandler.UnSubscribe(value); }
        }

        private ParallelActionHandler CalculateElementsHandler_x;
        protected void OnCalculateElements_x(bool parallel = false)
        {
            CalculateElementsHandler_x.Raise(parallel);
        }
        public event Action CalculateElements_x
        {
            add { CalculateElementsHandler_x.Subscribe(value); }
            remove { CalculateElementsHandler_x.UnSubscribe(value); }
        }

        private ParallelActionHandler CalculateElementsHandler_θ;
        protected void OnCalculateElements_θ(bool parallel = false)
        {
            CalculateElementsHandler_θ.Raise(parallel);
        }
        public event Action CalculateElements_θ
        {
            add { CalculateElementsHandler_θ.Subscribe(value); }
            remove { CalculateElementsHandler_θ.UnSubscribe(value); }
        }

        private ParallelActionHandler CalculateConstraintsHandler_x;
        protected void OnCalculateConstraints_x(bool parallel = false)
        {
            CalculateConstraintsHandler_x.Raise(parallel);
        }
        public event Action CalculateConstraints_x
        {
            add { CalculateConstraintsHandler_x.Subscribe(value); }
            remove { CalculateConstraintsHandler_x.UnSubscribe(value); }
        }

        private ParallelActionHandler CalculateConstraintsHandler_θ;
        protected void OnCalculateConstraints_θ(bool parallel = false)
        {
            CalculateConstraintsHandler_θ.Raise(parallel);
        }
        public event Action CalculateConstraints_θ
        {
            add { CalculateConstraintsHandler_θ.Subscribe(value); }
            remove { CalculateConstraintsHandler_θ.UnSubscribe(value); }
        }


        // RELAX
        private int nx, nθ;

        private double dt = 1.0;
        public double Ec_x, Ec_x_0, Ec_x_1, Ec_x_lim;
        public double Ec_θ, Ec_θ_0, Ec_θ_1, Ec_θ_lim;

        // STEP MOVE
        private MVector[][] dx;
        private MVector[][] dθ;

        // ALIAS
        private MVector[][] R_x;
        private MVector[][] R_θ;
        private double[][] lm_x, lm_θ;
        private MVector[][] v_x, v_θ;
        private MVector[][] a_x, a_θ;

        public Beam[] elements_x, elements_θ;
        private Support[] constraints_x;
        public Link[] links_x, links_θ;

        // BUILD
        public KDRSolver(IEnumerable<Beam> elements, IEnumerable<Support> constraints, IEnumerable<Link> links, int maxIteration = 1000, double Ec_x_lim = 1e-12, double Ec_θ_lim = 1e-10)
        {
            MaxIteration = maxIteration;
            this.Ec_x_lim = Ec_x_lim;
            this.Ec_θ_lim = Ec_θ_lim;

            CurrentIteration_x = 0;
            CurrentIteration_θ = 0;
            NumberOfKineticPeaks_x = 0;
            NumberOfKineticPeaks_θ = 0;

            ParallelOptions = new ParallelOptions();
            IsParallelModeEnabled = false;

            InitElementsHandler = new ParallelActionHandler(ParallelOptions, IsParallelModeEnabled);
            InitConstraintsHandler = new ParallelActionHandler(ParallelOptions, IsParallelModeEnabled);
            CalculateElementsHandler_x = new ParallelActionHandler(ParallelOptions, IsParallelModeEnabled);
            CalculateElementsHandler_θ = new ParallelActionHandler(ParallelOptions, IsParallelModeEnabled);
            CalculateConstraintsHandler_x = new ParallelActionHandler(ParallelOptions, IsParallelModeEnabled);
            CalculateConstraintsHandler_θ = new ParallelActionHandler(ParallelOptions, IsParallelModeEnabled);

            Build(elements, constraints, links);
            SolverSetup();
        }
        private void Build(IEnumerable<Beam> elements, IEnumerable<Support> constraints, IEnumerable<Link> links)
        {
            // ELEMENTS
            var elements_x = new List<Beam>();
            var elements_θ = new List<Beam>();

            foreach (var element in elements)
            {
                var beam = element as Beam_4DOF_D;

                elements_x.Add(beam);
                CalculateElements_x += beam.Calculate_x;
                InitElements += beam.Init;

                if (beam.IsTorsionCapable)
                {
                    elements_θ.Add(element);
                    CalculateElements_θ += beam.Calculate_θ;
                }
            }

            this.elements_x = elements_x.ToArray();
            this.elements_θ = elements_θ.ToArray();

            // CONSTRAINTS
            var constraints_x = new List<Support>();

            // INIT
            foreach (var cst in constraints)
            {
                constraints_x.Add(cst);
                InitConstraints += cst.Init;
            }

            this.constraints_x = constraints_x.ToArray();


            // LINKS
            var links_x = new List<Link>();
            var links_θ = new List<Link>();
            foreach (var link in links)
            {
                links_x.Add(link);
                InitConstraints += link.Init;

                if (link.IsTorsionCapable)
                {
                    links_θ.Add(link);
                }
            }

            this.links_x = links_x.ToArray();
            this.links_θ = links_θ.ToArray();

        }
        private void SolverSetup()
        {
            nx = elements_x.Length;
            nθ = elements_x.Length;

            lm_x = new double[nx][];
            R_x = new MVector[nx][];
            v_x = new MVector[nx][];
            a_x = new MVector[nx][];
            dx = new MVector[nx][];

            for (int i = 0; i < nx; i++)
            {
                var el = (Beam)elements_x[i];
                int n = el.Nv;
                lm_x[i] = el.Mx;
                R_x[i] = el.Rx;
                v_x[i] = el.Vx;
                a_x[i] = el.Ax;
                dx[i] = new MVector[n];
            }

            lm_θ = new double[nθ][];        
            R_θ = new MVector[nθ][];           
            v_θ = new MVector[nθ][];
            a_θ = new MVector[nθ][];
            dθ = new MVector[nθ][];

            for (int i = 0; i < nθ; i++)
            {
                var el = (Beam)elements_θ[i];
                int n = el.Nv;
                lm_θ[i] = el.Mθ;
                R_θ[i] = el.Rθ;
                v_θ[i] = el.Vθ;
                a_θ[i] = el.Aθ;
                dθ[i] = new MVector[n];
            }

            // Default action on Peak
            OnEnergyPeak_x += OnKineticEnergyPeak_x;
            OnEnergyPeak_θ += OnKineticEnergyPeak_θ;

            Init();

            Reset_x();
            Reset_θ();
        }

        // INIT
        private void Init()
        {
            CurrentIteration_x = 0;
            CurrentIteration_θ = 0;
            NumberOfKineticPeaks_x = 0;
            NumberOfKineticPeaks_θ = 0;

            // apply applied displacements
            // get boundary references (after applied displacements)
            OnInitConstraints();
            OnInitElements();
        }

        // RUN
        public void Run()
        {
            Run_x();
            Run_θ();
        }
        public void Run(int N)
        {
            State = SolverState.Running;
            int targetIteration = CurrentIteration_x + N;

            while (CurrentIteration_x < targetIteration)
            {
                Run();

                if (Ec_x < Ec_x_lim && Ec_θ < Ec_θ_lim)
                {
                    State = SolverState.Converged;
                    OnConvergence(this);
                    break;
                }

                if (CurrentIteration_x == MaxIteration)
                {
                    State = SolverState.Ended;
                    OnNotConvergence(this);
                    break;
                }
            }
        }

        // TRANSLATION
        private void Reset_x()
        {
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            UpdateDeformedConfig_x();

            // LUMPED MASS
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].Update_lm_x(ref lm_x[ei]);
            }
            //for (int ei = 0; ei < nθ; ei++)
            //{
            //    elements_θ[ei].Update_lm_θ(ref lm_θ[ei]);
            //}

            // TRANSFER MX
            foreach (var lk in links_x) { lk.Transfer_Mx(); } // calcul des projections à partir des positions actuelles 

            // VELOCITY
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    var a = (1 / lm_x[ei][nj]) * R_x[ei][nj];
                    a_x[ei][nj] = a;
                    v_x[ei][nj] = (0.5 * dt) * a;
                    //v_x[ei][nj] = (0.5 * dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // KINETIC ENERGY
            ComputeEc_x();
            Ec_x_1 = Ec_x;
            Ec_x_0 = 0.0;
        }
        private void Run_x()
        {
            // MOVE TO NEW POSITION ACCORDING TO CURRENT VELOCITY
            Move_x(dt);
            CurrentIteration_x++;

            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            UpdateDeformedConfig_x();

            // VELOCITY
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    var a = (1/ lm_x[ei][nj]) * R_x[ei][nj];
                    a_x[ei][nj] = a;
                    v_x[ei][nj] += dt * a;
                }
            }

            // KINETIC ENERGY
            ComputeEc_x();

            // PIC DETECTION
            if (Ec_x >= Ec_x_1) // NO PIC
            {
                Ec_x_0 = Ec_x_1;
                Ec_x_1 = Ec_x;
            }
            else // KINETIC PIC REACHED
            {
                // INTERPOLATION & POSITION
                InterpolateEc_x(Ec_x_0, Ec_x_1, Ec_x);

                NumberOfKineticPeaks_x++;
                OnEnergyPeak_x(this);

                // RESET
                Reset_x();
            }
        }
        private void Move_x(double dt)
        {
            for (int ei = 0; ei < nx; ei++)
            {
                for (int vj = 0; vj < elements_x[ei].Nv; vj++)
                {
                    dx[ei][vj] = dt * v_x[ei][vj];
                }
                elements_x[ei].Move_x(dx[ei]);
            }
        }
        private void ComputeEc_x()
        {
            Ec_x = 0.0;
            for (int ei = 0; ei < nx; ei++)
            {
                for (int vj = 0; vj < elements_x[ei].Nv; vj++)
                {
                    Ec_x += lm_x[ei][vj] * (v_x[ei][vj] * v_x[ei][vj]);
                }
            }
            Ec_x = 0.5 * Ec_x;
        }
        private void InterpolateEc_x(double E0, double E1, double E2)
        {
            // COMPUTE PIC INTERPOLATION
            double q = (E2 - E1) / (E0 - 2 * E1 + E2);
            Ec_x = E1 + (1 - 2 * q) / 8 * (E2 - E0);

            // COMPUTE BACKWARD VELOCITY
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    v_x[ei][nj] -= (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // COMPUTE BACKWARD POSITION
            Move_x(-q * dt);
        }

        // ROTATION
        private void Reset_θ()
        {
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            UpdateDeformedConfig_θ();

            // LUMPED MASS
            for (int ei = 0; ei < nθ; ei++)
            {
                elements_θ[ei].Update_lm_θ(ref lm_θ[ei]);
            }

            // TRANSFER MX
            foreach (var lk in links_θ) { lk.Transfer_Mθ(); } // calcul des projections à partir des positions actuelles 

            // VELOCITY
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    // in LCS
                    var a = (1 / lm_θ[ei][nj]) * R_θ[ei][nj];
                    a_θ[ei][nj] = a;
                    v_θ[ei][nj] = (0.5 * dt) * a;
                }
            }

            // KINETIC ENERGY
            ComputeEc_θ();
            Ec_θ_1 = Ec_θ;
            Ec_θ_0 = 0;
        }
        private void Run_θ()
        {
            // MOVE TO NEW POSITION ACCORDING TO CURRENT VELOCITY
            Move_θ(dt);
            CurrentIteration_θ++;

            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            UpdateDeformedConfig_θ();

            // VELOCITY
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    // in LCS
                    var a = (1 / lm_θ[ei][nj]) * R_θ[ei][nj];
                    a_θ[ei][nj] = a;
                    v_θ[ei][nj] += dt * a;
                }
            }

            // KINETIC ENERGY
            ComputeEc_θ();

            // PIC DETECTION
            if (Ec_θ > Ec_θ_1 || Ec_θ == 0) // NO PIC (le cas Ec_θ == 0 correspond à un état strictement immobile, donc pas de pic)
            {
                Ec_θ_0 = Ec_θ_1;
                Ec_θ_1 = Ec_θ;
            }
            else // KINETIC PIC REACHED
            {
                // INTERPOLATION
                InterpolateEc_θ(Ec_θ_0, Ec_θ_1, Ec_θ);
                NumberOfKineticPeaks_θ++;
                OnEnergyPeak_θ(this);

                // RESET
                Reset_θ();
            }
        }
        private void Move_θ(double dt)
        {
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int vj = 0; vj < elements_θ[ei].Nv; vj++)
                {
                    dθ[ei][vj] = dt * v_θ[ei][vj];
                }
                elements_θ[ei].Move_θ(dθ[ei]);
            }
        }
        private void ComputeEc_θ()
        {
            // devrait etre demandé à l'élément de calculer son énergie cinétique.
            Ec_θ = 0.0;
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int vj = 0; vj < elements_θ[ei].Nv; vj++)
                {
                    Ec_θ += lm_θ[ei][vj] * (v_θ[ei][vj].Z * v_θ[ei][vj].Z); // => enlever la projection sur d3 pour la relaxation 6DOF
                }
            }
            Ec_θ = 0.5 * Ec_θ;
        }
        private void InterpolateEc_θ(double E0, double E1, double E2)
        {
            // COMPUTE PIC INTERPOLATION
            double q = (E2 - E1) / (E0 - 2 * E1 + E2);
            Ec_θ = E1 + (1 - 2 * q) / 8 * (E2 - E0);

            // COMPUTE BACKWARD VELOCITY 
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    v_θ[ei][nj].Z -= (dt / lm_θ[ei][nj]) * R_θ[ei][nj].Z;
                }
            }

            // COMPUTE BACKWARD DISPLACEMENT
            Move_θ(-q * dt);
        }

        // ELEMENTS : UPDATE CONFIG WITH CONSTRAINTS
        private void UpdateDeformedConfig_x()
        {
            // Now Geometry is Locked
            foreach (var lk in links_x) { lk.Calculate_x(); } // calcul des projections à partir des positions actuelles 
            foreach (var lk in links_x) { lk.Transfer_Rx(); } // calcul des projections à partir des positions actuelles 

            //for (int i = 0; i < elements_x.Length; i++)
            //{
            //    elements_x[i].Calculate_x();
            //}

            OnCalculateElements_x();
        }
        private void UpdateDeformedConfig_θ()
        {
            // Now Geometry is Locked
            foreach (var lk in links_θ) { lk.Calculate_θ(); }
            foreach (var lk in links_x) { lk.Transfer_Rθ(); }

            //for (int i = 0; i < elements_θ.Length; i++){ elements_θ[i].Calculate_θ(); }

            OnCalculateElements_θ();
        }

        private static void OnKineticEnergyPeak_x(KDRSolver solver)
        {
            // do lump mass update here ?
            Debug.WriteLine("KineticEnergyPeak Reach for x at iteration = " + solver.CurrentIteration_x);
        }
        private static void OnKineticEnergyPeak_θ(KDRSolver solver)
        {
            // do lump mass update here ?
            Debug.WriteLine("KineticEnergyPeak Reach for x at iteration = " + solver.CurrentIteration_x);
        }
    }
}
