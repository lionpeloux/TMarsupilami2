using System;
using System.Collections.Generic;
using System.Diagnostics;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    public enum SolverState : int
    {
        Running,
        Converged,
        Ended,
        Aborted
    }

    public class KDRSolver : IDRSolver
    {
        public SolverState State { get; protected set; }
        public int MaxIteration { get; protected set; }

        public int CurrentIteration_x { get; protected set; }
        public int CurrentIteration_θ { get; protected set; }

        public int NumberOfKineticPeaks_x { get; protected set; }
        public int NumberOfKineticPeaks_θ { get; protected set; }

        public List<double> KineticEnergy_x { get { return Ec_x_history; } }
        public List<double> KineticEnergy_θ { get { return Ec_x_history; } }

        private Action<KDRSolver> del_x;
        private Action<KDRSolver> del_θ;

        // RELAX
        private int nx, nθ;
        private int index;

        private double dt = 1.0;

        private List<double> Ec_x_history;
        private double Ec_x, Ec_x_0, Ec_x_1, Ec_x_lim;

        private List<double> Ec_θ_history;
        private double Ec_θ, Ec_θ_0, Ec_θ_1, Ec_θ_lim;

        private MVector[][] R_x;
        private double[][] R_θ;
        private double[][] lm_x, lm_θ;
        private MVector[][] v_x;
        private double[][] v_θ;
        
        private IDRElement[] elements_x, elements_θ;
        private IDRConstraint[] constraints, apd;

        // BUILD
        private void Dispatch(IEnumerable<IDRElement> elements, IEnumerable<IDRConstraint> constraints)
        {
            var tmp_x = new List<IDRElement>();
            var tmp_θ = new List<IDRElement>();

            foreach (var element in elements)
            {
                if (element.IsTorsionCapable)
                {
                    tmp_θ.Add(element);
                    tmp_x.Add(element);
                }
                else
                {
                    tmp_x.Add(element);
                }
            }

            elements_x = tmp_x.ToArray();
            elements_θ = tmp_θ.ToArray();
        }
        private void SolverSetup()
        {
            nx = elements_x.Length;
            nθ = elements_x.Length;

            lm_x = new double[nx][];
            R_x = new MVector[nx][];
            v_x = new MVector[nx][];

            for (int i = 0; i < nx; i++)
            {
                var el = elements_x[i];
                int n = el.Nv;
                lm_x[i] = new double[n];
                R_x[i] = el.Rx;
                v_x[i] = new MVector[n];
            }

            lm_θ = new double[nθ][];        
            R_θ = new double[nθ][];           
            v_θ = new double[nθ][];

            for (int i = 0; i < nθ; i++)
            {
                var el = elements_θ[i];
                int n = el.Nv;
                lm_θ[i] = new double[n];
                R_θ[i] = el.Rθ;
                v_θ[i] = new double[n];
            }

            // Default action on Peak
            del_x = OnKineticEnergyPeak_x;
            del_x = OnKineticEnergyPeak_θ;
        }

        // INIT
        private void Init()
        {
            CurrentIteration_x = 0;
            CurrentIteration_θ = 0;
            NumberOfKineticPeaks_x = 0;
            NumberOfKineticPeaks_θ = 0;

            Reset_x();
            Reset_θ();

            // the model is set up in it's initial configuration
            // constraints are applied and resultant forces and moments are computed
            //Rhino.RhinoApp.WriteLine("INIT_X : " + CurrentIteration_x);

            // apply applied displacements
            foreach (var cst in apd) { cst.Init(); }

            // get boundary references (after applied displacements)
            foreach (var cst in constraints) { cst.Init(); }

            // update config
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
                    this.State = SolverState.Converged;
                    break;
                }

                if (CurrentIteration_x == MaxIteration)
                {
                    this.State = SolverState.Ended;
                    break;
                }
            }
        }

        // TRANSLATION
        private void Reset_x()
        {
            //Rhino.RhinoApp.WriteLine("###### RESET X");
            //Rhino.RhinoApp.WriteLine("Ec_x[" + numPic_x + "] = " + string.Format("{0:E2}", Ec_x));

            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateResultantNodalMoment();
                elements_x[ei].UpdateResultantNodalForce();
            }

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
                    v_x[ei][nj] = 0.5 * (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_x = 0.0;
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    Ec_x += 0.5 * lm_x[ei][nj] * (v_x[ei][nj] * v_x[ei][nj]);
                }
            }
            Ec_x_history.Add(Ec_x);
            Ec_x_1 = Ec_x;
            Ec_x_0 = 0.0;

            // POSITION
            for (int ei = 0; ei < nx; ei++)
            {
                MVector[] dx = new MVector[elements_x[ei].Nv];
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    dx[nj] = dt * v_x[ei][nj];
                }
                elements_x[ei].Move(dx);
                //elements[ei].GetDeformedConfig_x();
            }
            UpdateDeformedConfig_x();

            CurrentIteration_x++;
        }
        private void Run_x()
        {
            //Rhino.RhinoApp.WriteLine("###### RUN X");
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateResultantNodalMoment();
                elements_x[ei].UpdateResultantNodalForce();
            }

            // VELOCITY
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    v_x[ei][nj] += (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_x = 0.0;
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    Ec_x += 0.5 * lm_x[ei][nj] * (v_x[ei][nj] * v_x[ei][nj]);
                }
            }

            // PIC DETECTION
            if (Ec_x >= Ec_x_1) // NO PIC
            {
                Ec_x_history.Add(Ec_x);
                Ec_x_0 = Ec_x_1;
                Ec_x_1 = Ec_x;

                // POSITION
                for (int ei = 0; ei < nx; ei++)
                {
                    MVector[] dx = new MVector[elements_x[ei].Nv];
                    for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                    {
                        dx[nj] = dt * v_x[ei][nj];
                    }
                    elements_x[ei].Move(dx);
                    //elements[ei].GetDeformedConfig_x();
                }
                UpdateDeformedConfig_x();
                del_x(this);
                CurrentIteration_x++;
            }
            else // KINETIC PIC REACHED
            {
                // INTERPOLATION & POSITION
                InterpolateEc_x(Ec_x_0, Ec_x_1, Ec_x);
                Ec_x_history[Ec_x_history.Count - 1] = Ec_x;

                NumberOfKineticPeaks_x++;

                // RESET
                Reset_x();
            }
        }
        private void InterpolateEc_x(double E0, double E1, double E2)
        {
            //Rhino.RhinoApp.WriteLine("###### INTERP X");
            // COMPUTE PIC INTERPOLATION
            double q = (E2 - E1) / (E0 - 2 * E1 + E2);

            // COMPUTE BACKWARD VELOCITY
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    v_x[ei][nj] -= (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // COMPUTE BACKWARD DISPLACEMENT AND MOVE
            for (int ei = 0; ei < nx; ei++)
            {
                MVector[] dx = new MVector[elements_x[ei].Nv];
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    dx[nj] = -q * dt * v_x[ei][nj];
                }
                elements_x[ei].Move(dx);
                //elements[ei].GetDeformedConfig_x();
            }
            UpdateDeformedConfig_x();

            // COMPUTE ENERGY PIC
            Ec_x = E1 + (1 - 2 * q) / 8 * (E2 - E0);
        }

        // ROTATION (QUASISTATIC)
        private void Reset_θ()
        {
            //Rhino.RhinoApp.WriteLine("###### RESET θ");

            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < nθ; ei++)
            {
                elements_θ[ei].UpdateResultantNodalMoment();
                elements_θ[ei].UpdateResultantNodalForce();
            }

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
                    v_θ[ei][nj] = 0.5 * (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_θ = 0.0;
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    Ec_θ += 0.5 * lm_θ[ei][nj] * (v_θ[ei][nj] * v_θ[ei][nj]);
                }
            }
            Ec_θ_history.Add(Ec_θ);
            Ec_θ_1 = Ec_θ;
            Ec_θ_0 = 0;

            // POSITION
            for (int ei = 0; ei < nθ; ei++)
            {
                double[] dθ = new double[elements_θ[ei].Nv];
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    dθ[nj] = dt * v_θ[ei][nj];
                    //Rhino.RhinoApp.WriteLine("dθ[" + nj + "] = " + dθ[nj]);
                }
                elements_θ[ei].Move(dθ);
                //elements[ei].GetDeformedConfig_θ();
            }
            UpdateDeformedConfig_θ();

            CurrentIteration_θ++;
        }
        private void Run_θ()
        {
            //Rhino.RhinoApp.WriteLine("###### RUN θ");
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < nθ; ei++)
            {
                elements_θ[ei].UpdateResultantNodalMoment();
                elements_θ[ei].UpdateResultantNodalForce();
            }

            // VELOCITY
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    v_θ[ei][nj] += (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_θ = 0.0;
            for (int ei = 0; ei < nθ; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    Ec_θ += 0.5 * lm_θ[ei][nj] * (v_θ[ei][nj] * v_θ[ei][nj]);
                }
            }

            // PIC DETECTION
            if (Ec_θ > Ec_θ_1 || Ec_θ == 0) // NO PIC (le cas Ec_θ == 0 correspond à un état strictement immobile, donc pas de pic)
            {
                Ec_θ_history.Add(Ec_θ);
                Ec_θ_0 = Ec_θ_1;
                Ec_θ_1 = Ec_θ;

                // POSITION
                for (int ei = 0; ei < nθ; ei++)
                {
                    double[] dθ = new double[elements_θ[ei].Nv];
                    for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                    {
                        dθ[nj] = dt * v_θ[ei][nj];
                    }
                    elements_θ[ei].Move(dθ);
                    //elements[ei].GetDeformedConfig_θ();
                }
                UpdateDeformedConfig_θ();

                CurrentIteration_θ++;
            }
            else // KINETIC PIC REACHED
            {
                // INTERPOLATION
                InterpolateEc_θ(Ec_θ_0, Ec_θ_1, Ec_θ);
                Ec_θ_history[Ec_θ_history.Count - 1] = Ec_θ;

                NumberOfKineticPeaks_θ++;

                // RESET
                Reset_θ();
            }
        }
        private void InterpolateEc_θ(double E0, double E1, double E2)
        {
            //Rhino.RhinoApp.WriteLine("###### INTERP θ");
            // COMPUTE PIC INTERPOLATION
            double q = (E2 - E1) / (E0 - 2 * E1 + E2);

            // COMPUTE BACKWARD VELOCITY 
            for (int ei = 0; ei < nx; ei++)
            {
                for (int nj = 0; nj < elements_θ[ei].Nv; nj++)
                {
                    v_θ[ei][nj] -= (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // COMPUTE BACKWARD DISPLACEMENT
            for (int ei = 0; ei < nx; ei++)
            {
                double[] dθ = new double[elements_x[ei].Nv];
                for (int nj = 0; nj < elements_x[ei].Nv; nj++)
                {
                    dθ[nj] = -q * dt * v_θ[ei][nj];
                }
                elements_x[ei].Move(dθ);
                //elements[ei].GetDeformedConfig_θ();
            }
            UpdateDeformedConfig_θ();

            // ENERGY PIC
            Ec_θ = E1 + (1 - 2 * q) / 8 * (E2 - E0);
        }

        // ELEMENTS : UPDATE CONFIG WITH CONSTRAINTS
        private void UpdateDeformedConfig_x()
        {

            // update centerline properties (e,l,ll,t)
            for (int ei = 0; ei < nx; ei++) { elements_x[ei].UpdateCenterlineProperties(); }

            // enforce tangent constraints (t, κb)
            foreach (var cst in constraints) { cst.Enforce_Mr(); }

            // update curvature binormal (κb)
            for (int ei = 0; ei < nx; ei++)
            { elements_x[ei].UpdateCurvatureBinormal(); }

            // update material frames
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateMaterialFrame();
                //elements[ei].GetMidEdgeQuantities();
            }

            // update internal bending and twisting moments
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateBendingMoment();
                elements_x[ei].UpdateTwistingMoment();
            }

            // update internal axial and shear forces
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateAxialForce();
                elements_x[ei].UpdateShearForce();
            }

            // update resulting internal nodal force and internal nodal moment
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateInternalNodalMoment();
                elements_x[ei].UpdateInternalNodalForce();
            }

            // enforce nodal force and moment contraints
            foreach (var cst in constraints) { cst.Enforce_Qr(); }
            foreach (var cst in constraints) { cst.Enforce_Fr(); }
        }
        private void UpdateDeformedConfig_θ()
        {
            // update internal bending and twisting moments
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateBendingMoment();
                elements_x[ei].UpdateTwistingMoment();
            }

            // update internal shear forces
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateShearForce();
            }

            // update resulting internal nodal force and internal nodal moment
            for (int ei = 0; ei < nx; ei++)
            {
                elements_x[ei].UpdateInternalNodalMoment();
                elements_x[ei].UpdateInternalNodalForce();
            }

            // enforce nodal force and moment contraints
            foreach (var cst in constraints) { cst.Enforce_Qr(); }
            foreach (var cst in constraints) { cst.Enforce_Fr(); }
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
