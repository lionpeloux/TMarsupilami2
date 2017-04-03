using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{

    public class KDRSolver2 : IDRSolver
    {
        public bool IsParallelModeEnabled { get; private set; }
        public ParallelOptions ParallelOptions { get; private set; }

        public SolverState State { get; protected set; }
        public int MaxIteration { get; protected set; }

        public int CurrentIteration_x { get; protected set; }
        public int CurrentIteration_θ { get; protected set; }

        public int NumberOfKineticPeaks_x { get; protected set; }
        public int NumberOfKineticPeaks_θ { get; protected set; }

        public event Action<KDRSolver2> OnEnergyPeak_x;
        public event Action<KDRSolver2> OnEnergyPeak_θ;
        public event Action<KDRSolver2> OnConvergence;

        private Cluster Initialize;
        
        // 3 and 4 DOF elements : X
        private Cluster Update_x_CenterlineProperties;
        private Cluster Update_x_CurvatureBinormal;
        private Cluster Update_x_MaterialFrame;
        private Cluster Update_x_TwistingMoment;
        private Cluster Update_x_BendingMoment;
        private Cluster Update_x_AxialForce;
        private Cluster Update_x_ShearForce;
        private Cluster Update_x_InternalNodalMoment;
        private Cluster Update_x_InternalNodalForce;
        private Cluster Update_x_ResultantNodalMoment;
        private Cluster Update_x_ResultantNodalForce;

        // 4 DOF : θ
        private Cluster Update_θ_CenterlineProperties;
        private Cluster Update_θ_CurvatureBinormal;
        private Cluster Update_θ_MaterialFrame;
        private Cluster Update_θ_TwistingMoment;
        private Cluster Update_θ_BendingMoment;
        private Cluster Update_θ_AxialForce;
        private Cluster Update_θ_ShearForce;
        private Cluster Update_θ_InternalNodalMoment;
        private Cluster Update_θ_InternalNodalForce;
        private Cluster Update_θ_ResultantNodalMoment;
        private Cluster Update_θ_ResultantNodalForce;

        // RELAX
        private int nx, nθ;

        private double dt = 1.0;
        public double Ec_x, Ec_x_0, Ec_x_1, Ec_x_lim;
        public double Ec_θ, Ec_θ_0, Ec_θ_1, Ec_θ_lim;

        // STEP MOVE
        private MVector[][] dx;
        private double[][] dθ;

        // ALIAS
        private MVector[][] R_x;
        private double[][] R_θ;
        private double[][] lm_x, lm_θ;
        private MVector[][] v_x, v_θ;
        private MVector[][] a_x, a_θ;

        private IDRElement[] elements_x, elements_θ;
        private IDRConstraint[] constraints, apd;

        // BUILD
        public KDRSolver2(IEnumerable<IDRElement> elements, IEnumerable<IDRConstraint> constraints, int maxIteration = 1000, double Ec_x_lim = 1e-12, double Ec_θ_lim = 1e-10)
        {
            MaxIteration = maxIteration;
            this.Ec_x_lim = Ec_x_lim;
            this.Ec_θ_lim = Ec_θ_lim;

            CurrentIteration_x = 0;
            CurrentIteration_θ = 0;
            NumberOfKineticPeaks_x = 0;
            NumberOfKineticPeaks_θ = 0;

            Build(elements, constraints);
            SolverSetup();
        }
        private void Build(IEnumerable<IDRElement> elements, IEnumerable<IDRConstraint> constraints)
        {
            var tmp_x = new List<IDRElement>();
            var tmp_θ = new List<IDRElement>();

            ParallelOptions = new ParallelOptions();
            IsParallelModeEnabled = false;

            Initialize = new Cluster(ParallelOptions, IsParallelModeEnabled);
            

            Update_x_CenterlineProperties = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_CurvatureBinormal = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_MaterialFrame = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_TwistingMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_BendingMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_AxialForce = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_ShearForce = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_InternalNodalMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_InternalNodalForce = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_ResultantNodalMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_x_ResultantNodalForce = new Cluster(ParallelOptions, IsParallelModeEnabled);

            Update_θ_CenterlineProperties = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_CurvatureBinormal = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_MaterialFrame = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_TwistingMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_BendingMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_AxialForce = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_ShearForce = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_InternalNodalMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_InternalNodalForce = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_ResultantNodalMoment = new Cluster(ParallelOptions, IsParallelModeEnabled);
            Update_θ_ResultantNodalForce = new Cluster(ParallelOptions, IsParallelModeEnabled);

            //
            foreach (var cst in constraints) { Initialize.Subscribe(cst.Init); }


            foreach (var element in elements)
            {
                tmp_x.Add(element);

                Update_x_CenterlineProperties.Subscribe(element.UpdateCenterlineProperties);
                Update_x_CurvatureBinormal.Subscribe(element.UpdateCurvatureBinormal);
                Update_x_MaterialFrame.Subscribe(element.UpdateMaterialFrame);

                Update_x_TwistingMoment.Subscribe(element.UpdateTwistingMoment);
                Update_x_BendingMoment.Subscribe(element.UpdateBendingMoment);
                Update_x_AxialForce.Subscribe(element.UpdateAxialForce);
                Update_x_ShearForce.Subscribe(element.UpdateShearForce);

                Update_x_InternalNodalMoment.Subscribe(element.UpdateInternalNodalMoment);
                Update_x_InternalNodalForce.Subscribe(element.UpdateInternalNodalForce);

                Update_x_ResultantNodalMoment.Subscribe(element.UpdateResultantNodalMoment);
                Update_x_ResultantNodalForce.Subscribe(element.UpdateResultantNodalForce);

                if (element.IsTorsionCapable)
                {
                    tmp_θ.Add(element);

                    Update_θ_CenterlineProperties.Subscribe(element.UpdateCenterlineProperties);
                    Update_θ_CurvatureBinormal.Subscribe(element.UpdateCurvatureBinormal);
                    Update_θ_MaterialFrame.Subscribe(element.UpdateMaterialFrame);

                    Update_θ_TwistingMoment.Subscribe(element.UpdateTwistingMoment);
                    Update_θ_BendingMoment.Subscribe(element.UpdateBendingMoment);
                    Update_θ_AxialForce.Subscribe(element.UpdateAxialForce);
                    Update_θ_ShearForce.Subscribe(element.UpdateShearForce);

                    Update_θ_InternalNodalMoment.Subscribe(element.UpdateInternalNodalMoment);
                    Update_θ_InternalNodalForce.Subscribe(element.UpdateInternalNodalForce);

                    Update_θ_ResultantNodalMoment.Subscribe(element.UpdateResultantNodalMoment);
                    Update_θ_ResultantNodalForce.Subscribe(element.UpdateResultantNodalForce);
                }
            }

            this.elements_x = tmp_x.ToArray();
            this.elements_θ = tmp_θ.ToArray();

            this.constraints = constraints.ToArray();
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
                var el = elements_x[i];
                int n = el.Nv;
                lm_x[i] = el.LMx;
                R_x[i] = el.Rx;
                v_x[i] = el.Vx;
                a_x[i] = el.Ax;
                dx[i] = new MVector[n];
            }

            lm_θ = new double[nθ][];        
            R_θ = new double[nθ][];           
            v_θ = new MVector[nθ][];
            a_θ = new MVector[nθ][];
            dθ = new double[nθ][];

            for (int i = 0; i < nθ; i++)
            {
                var el = elements_θ[i];
                int n = el.Nv;
                lm_θ[i] = el.LMθ;
                R_θ[i] = el.Rθ;
                v_θ[i] = el.Vθ;
                a_θ[i] = el.Aθ;
                dθ[i] = new double[n];
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

            // the model is set up in it's initial configuration
            // constraints are applied and resultant forces and moments are computed
            //Rhino.RhinoApp.WriteLine("INIT_X : " + CurrentIteration_x);

            // apply applied displacements
            //foreach (var cst in apd) { cst.Init(); }

            // get boundary references (after applied displacements)
            Initialize.Call();

            //// update config
            UpdateDeformedConfig_x();
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
                    break;
                }
            }
        }

        // TRANSLATION
        private void Reset_x()
        {
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            Update_x_ResultantNodalMoment.Call();
            Update_x_ResultantNodalForce.Call();

            // LUMPED MASS
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].Update_lm_x(ref lm_x[ei]);
            }

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

            // POSITION
            Move_x(dt);
            CurrentIteration_x++;
        }
        private void Run_x()
        {
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            Update_x_ResultantNodalMoment.Call();
            Update_x_ResultantNodalForce.Call();

            // VELOCITY
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    var a = (1/ lm_x[ei][nj]) * R_x[ei][nj];
                    a_x[ei][nj] = a;
                    v_x[ei][nj] += dt * a;
                    //v_x[ei][nj] += (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // KINETIC ENERGY
            ComputeEc_x();

            // PIC DETECTION
            if (Ec_x >= Ec_x_1) // NO PIC
            {
                Ec_x_0 = Ec_x_1;
                Ec_x_1 = Ec_x;

                // POSITION
                Move_x(dt);
                CurrentIteration_x++;
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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Move_x(double dt)
        {
            for (int ei = 0; ei < nx; ei++)
            {
                for (int vj = 0; vj < elements_x[ei].Nv; vj++)
                {
                    dx[ei][vj] = dt * v_x[ei][vj];
                }
                elements_x[ei].Move(dx[ei]);
            }
            UpdateDeformedConfig_x();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

            // COMPUTE BACKWARD DISPLACEMENT AND MOVE
            //for (int ei = 0; ei < nx; ei++)
            //{
            //    for (int nj = 0; nj < elements_x[ei].Nv; nj++)
            //    {
            //        dx[ei][nj] = -q * dt * v_x[ei][nj];
            //    }
            //    elements_x[ei].Move(dx[ei]);
            //}
            //UpdateDeformedConfig_x();
            Move_x(-q * dt);
        }

        // ROTATION (QUASISTATIC)
        private void Reset_θ()
        {
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            Update_θ_ResultantNodalMoment.Call();
            Update_θ_ResultantNodalForce.Call();

            // LUMPED MASS
            for (int ei = 0; ei < nθ; ei++)
            {
                elements_θ[ei].Update_lm_θ(ref lm_θ[ei]);
            }

            // VELOCITY
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    var a = (1 / lm_θ[ei][nj]) * R_θ[ei][nj];
                    a_θ[ei][nj].Z = a;
                    v_θ[ei][nj].Z = (0.5 * dt) * a;
                    //v_θ[ei][nj].Z = 0.5 * (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // KINETIC ENERGY
            ComputeEc_θ();
            Ec_θ_1 = Ec_θ;
            Ec_θ_0 = 0;

            // POSITION
            Move_θ(dt);
            CurrentIteration_θ++;
        }
        private void Run_θ()
        {
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            Update_θ_ResultantNodalMoment.Call();
            Update_θ_ResultantNodalForce.Call();

            // VELOCITY
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    var a = (1 / lm_θ[ei][nj]) * R_θ[ei][nj];
                    a_θ[ei][nj].Z = a;
                    v_θ[ei][nj].Z += dt * a;
                    //v_θ[ei][nj].Z += (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // KINETIC ENERGY
            ComputeEc_θ();

            // PIC DETECTION
            if (Ec_θ > Ec_θ_1 || Ec_θ == 0) // NO PIC (le cas Ec_θ == 0 correspond à un état strictement immobile, donc pas de pic)
            {
                Ec_θ_0 = Ec_θ_1;
                Ec_θ_1 = Ec_θ;

                // POSITION
                Move_θ(dt);
                CurrentIteration_θ++;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Move_θ(double dt)
        {
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int vj = 0; vj < elements_θ[ei].Nv; vj++)
                {
                    dθ[ei][vj] = dt * v_θ[ei][vj].Z;
                }
                elements_θ[ei].Move(dθ[ei]);
            }
            UpdateDeformedConfig_θ();         
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void ComputeEc_θ()
        {
            Ec_θ = 0.0;
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int vj = 0; vj < elements_θ[ei].Nv; vj++)
                {
                    Ec_θ += lm_θ[ei][vj] * (v_θ[ei][vj] * v_θ[ei][vj]);
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
                    v_θ[ei][nj].Z -= (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // COMPUTE BACKWARD DISPLACEMENT
            Move_θ(-q * dt);
        }

        // ELEMENTS : UPDATE CONFIG WITH CONSTRAINTS
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void UpdateDeformedConfig_x()
        {
            // update centerline properties (e,l,ll,t)
            Update_x_CenterlineProperties.Call();

            // enforce tangent constraints (t, κb)
            foreach (var cst in constraints) { cst.Enforce_Mr(); }

            // update curvature binormal (κb)
            Update_x_CurvatureBinormal.Call();

            // update material frames
            Update_x_MaterialFrame.Call();

            // update internal bending and twisting moments
            Update_x_BendingMoment.Call();
            Update_x_TwistingMoment.Call();

            // update internal axial and shear forces
            Update_x_AxialForce.Call();
            Update_x_ShearForce.Call();

            // update resulting internal nodal force and internal nodal moment
            Update_x_InternalNodalMoment.Call();
            Update_x_InternalNodalForce.Call();

            // enforce nodal force and moment contraints
            foreach (var cst in constraints) { cst.Enforce_Qr(); }
            foreach (var cst in constraints) { cst.Enforce_Fr(); }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void UpdateDeformedConfig_θ()
        {
            // update internal bending and twisting moments
            Update_θ_BendingMoment.Call();
            Update_θ_TwistingMoment.Call();

            // update internal shear forces
            Update_θ_ShearForce.Call();

            // update resulting internal nodal force and internal nodal moment
            Update_θ_InternalNodalMoment.Call();
            Update_θ_InternalNodalForce.Call();

            // enforce nodal force and moment contraints
            foreach (var cst in constraints) { cst.Enforce_Qr(); }
            foreach (var cst in constraints) { cst.Enforce_Fr(); }
        }

        private static void OnKineticEnergyPeak_x(KDRSolver2 solver)
        {
            // do lump mass update here ?
            Debug.WriteLine("KineticEnergyPeak Reach for x at iteration = " + solver.CurrentIteration_x);
        }
        private static void OnKineticEnergyPeak_θ(KDRSolver2 solver)
        {
            // do lump mass update here ?
            Debug.WriteLine("KineticEnergyPeak Reach for x at iteration = " + solver.CurrentIteration_x);
        }
    }
}
