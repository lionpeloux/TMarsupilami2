using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using TMarsupilami.TestModel.Dof4.Discontinuous;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;
using System.Diagnostics;

namespace TMarsupilami.Gh.Component
{
    public class _Comp_4DOF_DISC : GH_Component
    {
        private bool loop_reset = true;
        private bool loop_reset_cache = true;
        private int iteration_max;
        private bool is_base = true;

        // RELAX
        int N;
        private int index;
        private string info;
        private int numPic_x, numPic_θ;
        private int numIteration_x, numIteration_θ;
        private List<int> iterationHistory;
        private List<double> chronoHistory;

        private double dt = 1.0;
        private List<double> Ec_x_history, Ec_θ_history;
        private double Ec_x, Ec_x_0, Ec_x_1, Ec_x_lim;
        private double Ec_θ, Ec_θ_0, Ec_θ_1, Ec_θ_lim;

        private MFrame[][] frame;
        private MVector[][] R_x;
        private double[][] R_θ;
        private double[][] lm_x, lm_θ;
        private MVector[][] v_x;
        private double[][] v_θ;


        private List<double> b1, b2;
        private List<MFrame> sections_0, sections_i;
        private int bc_start, bc_end;
        private CurvedBeam[] elements;
        private Constraint[] constraints, apd;
        private List<SectionProperty> section_prop;
        private MaterialProperty material_prop;

        private Stopwatch watch;

        // CONSTRUCTOR
        public _Comp_4DOF_DISC()
            : base("4 DOF with Disc - simple beam", "4_DOF_DISC", "Single simple beam with various boundary conditions", "TMarsupilami", "Base Models")
        {
            sections_0 = new List<MFrame>();
            sections_i = new List<MFrame>();
            Ec_x_history = new List<double>();
            Ec_θ_history = new List<double>();

            N = 1;
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{AE1B948A-22D4-4252-A1F3-72890C0B8051}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        // PARAMETERS
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Rest position", "sections_0", "Beam's sections in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Initial position", "sections_i", "Beam's sections in initial position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);

            pManager.AddNumberParameter("Section b1", "b1", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Section b2", "b2", "", GH_ParamAccess.list);

            pManager.AddIntegerParameter("Boundary Condition at Start", "bc_start", "(FREE = 0, PINNED = 1, CLAMPED = 2)", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("Boundary Condition at End", "bc_end", "(FREE = 0, PINNED = 1, CLAMPED  =2)", GH_ParamAccess.item, 1);

            pManager.AddIntegerParameter("iteration_max", "N_max", "Total number of iterations to run", GH_ParamAccess.item, 10);
            pManager.AddBooleanParameter("reset", "reset", "Reset the engine", GH_ParamAccess.item, false);

            pManager.AddBooleanParameter("isBase", "isBase", "True pour revenir à la version sans discontinuités du model", GH_ParamAccess.item, true);

            pManager[4].Optional = false;
            pManager[5].Optional = false;
            pManager[6].Optional = false;

        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // tracking
            pManager.Register_StringParam("Info", "info", "");
            pManager.Register_IntegerParam("Index", "index", "");

            // rest config
            pManager.Register_DoubleParam("Length", "l_0", "Length of edge ei.");
            pManager.RegisterParam(new Param_MVector(), "Curvature Binormal", "kb_0", "Curvature binormal vector");
            pManager.Register_DoubleParam("Curvature 1", "k1_0", "Curvature around d1 material vector.");
            pManager.Register_DoubleParam("Curvature 2", "k2_0", "Curvature around d2 material vector.");
            pManager.Register_DoubleParam("Twist Angle", "twist_0", "Twist angle between to consecutive sections.");
            pManager.Register_DoubleParam("Rate of twist", "tau_0", "Rate of twist between to consecutive sections.");

            // deformed config
            pManager.AddParameter(new Param_MFrame(), "sections", "sections", "Actual configuration.", GH_ParamAccess.item);
            pManager.RegisterParam(new Param_MVector(), "e", "e", "Centerline edges.");
            pManager.RegisterParam(new Param_MVector(), "", "t", "");
            pManager.RegisterParam(new Param_MVector(), "", "t_g", "");
            pManager.RegisterParam(new Param_MVector(), "", "t_h_l", "");
            pManager.RegisterParam(new Param_MVector(), "", "t_h_r", "");

            pManager.Register_DoubleParam("", "l", "");
            pManager.Register_DoubleParam("", "ε", "");

            pManager.RegisterParam(new Param_MVector(), "", "kb_g", "");
            pManager.RegisterParam(new Param_MVector(), "", "kb_h_l", "");
            pManager.RegisterParam(new Param_MVector(), "", "kb_h_r", "");

            pManager.Register_DoubleParam("", "twist", "");
            pManager.Register_DoubleParam("", "τ", "");

            // external forces and moments
            pManager.RegisterParam(new Param_MVector(), "", "Fext", "");
            pManager.RegisterParam(new Param_MVector(), "", "Mext", "");
            pManager.RegisterParam(new Param_MVector(), "", "fext", "");
            pManager.RegisterParam(new Param_MVector(), "", "mext", "");

            pManager.RegisterParam(new Param_MVector(), "", "Fr", "");
            pManager.RegisterParam(new Param_MVector(), "", "Mr", "");

            // internal config
            pManager.RegisterParam(new Param_MVector(), "", "M_g", "");
            pManager.RegisterParam(new Param_MVector(), "", "M_h_l", "");
            pManager.RegisterParam(new Param_MVector(), "", "M_h_r", "");

            pManager.Register_DoubleParam("", "Q", "");
            pManager.Register_DoubleParam("", "Q_l", "");
            pManager.Register_DoubleParam("", "Q_r", "");

            pManager.RegisterParam(new Param_MVector(), "", "V_M_g", "");
            pManager.RegisterParam(new Param_MVector(), "", "V_M_h_l", "");
            pManager.RegisterParam(new Param_MVector(), "", "V_M_h_r", "");

            pManager.Register_DoubleParam("", "N", "");
            pManager.Register_DoubleParam("", "N_l", "");
            pManager.Register_DoubleParam("", "N_r", "");

            //// resultante
            pManager.RegisterParam(new Param_MVector(), "Rx_axial", "", ""); //16
            pManager.RegisterParam(new Param_MVector(), "Rx_shear_M", "", "");
            pManager.RegisterParam(new Param_MVector(), "Rx_shear_Q", "", "");
            pManager.Register_DoubleParam("Rθ_M", "", "");
            pManager.Register_DoubleParam("Rθ_Q", "", "");


            pManager.Register_DoubleParam("Ec_x", "Ec_x", "");
            pManager.Register_DoubleParam("Ec_θ", "Ec_θ", "");
            //pManager.Register_CurveParam("section_base", "section_base", "");

            pManager.Register_DoubleParam("lm_x", "lm_x", "");
            pManager.Register_DoubleParam("lm_θ", "lm_θ", "");

            pManager.RegisterParam(new Param_MVector(), "v_x", "v_x", "");
            pManager.Register_DoubleParam("v_θ", "v_θ", "");

            //pManager.RegisterParam(new Param_MVector(), "R_x", "R_x", "");
            //pManager.Register_DoubleParam("R_θ", "R_θ", "");



        }

        // SOLVER
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Rhino.RhinoApp.WriteLine("SOLVE INSTANCE");
            Rhino.RhinoApp.WriteLine("BRANCH : 1beam_simple");
            sections_0.Clear();
            sections_i.Clear();
            Ec_x_history.Clear();
            Ec_θ_history.Clear();

            b1 = new List<double>();
            b2 = new List<double>();

            if (!DA.GetDataList(0, sections_0)) { return; }
            if (!DA.GetDataList(1, sections_i)) { return; }

            if (!DA.GetDataList(2, b1)) { return; }
            if (!DA.GetDataList(3, b2)) { return; }

            if (!DA.GetData(4, ref bc_start)) { return; }
            if (!DA.GetData(5, ref bc_end)) { return; }

            if (!DA.GetData(6, ref iteration_max)) { return; }
            if (!DA.GetData(7, ref loop_reset)) { return; }

            if (!DA.GetData(8, ref is_base)) { return; }


            if (loop_reset == true) // Premier Calcul
            {
                loop_reset_cache = loop_reset;

                int n = sections_i.Count;
                section_prop = new List<SectionProperty>();

                // n-1 section definitions
                if (n == 1)
                {
                    var sprop = SectionProperty.RectangularSection(b1[0], b2[0]);
                    for (int i = 0; i < n - 1; i++)
                    {
                        section_prop.Add(sprop);
                    }
                }
                else
                {
                    for (int i = 0; i < n - 1; i++)
                    {
                        section_prop.Add(SectionProperty.RectangularSection(b1[i], b2[i]));
                    }
                }

                material_prop = new MaterialProperty(StandardMaterials.GFRP);
                DR_Relax(iteration_max);
            }


            //DA.SetData(0, info);
            DA.SetData(0, "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms");
            DA.SetData(1, numIteration_x);

            // rest configuration
            DA.SetDataList(2, elements[0].l_0);
            DA.SetDataList(3, elements[0].κb_0);
            DA.SetDataList(4, elements[0].κ1_0);
            DA.SetDataList(5, elements[0].κ2_0);
            DA.SetDataList(6, elements[0].twist_0);
            DA.SetDataList(7, elements[0].τ_0);

            // actual configuration
            DA.SetDataList(8, elements[0].MaterialFrame);
            DA.SetDataList(9, elements[0].e);
            DA.SetDataList(10, elements[0].t);
            DA.SetDataList(11, elements[0].t_g);
            DA.SetDataList(12, elements[0].t_h_l);
            DA.SetDataList(13, elements[0].t_h_r);

            DA.SetDataList(14, elements[0].l);
            DA.SetDataList(15, elements[0].ε);

            DA.SetDataList(16, elements[0].κb_g);
            DA.SetDataList(17, elements[0].κb_h_l);
            DA.SetDataList(18, elements[0].κb_h_r);
            DA.SetDataList(19, elements[0].twist);
            DA.SetDataList(20, elements[0].τ);

            DA.SetDataList(21, elements[0].GetFext(CoordinateSystem.Global));
            DA.SetDataList(22, elements[0].GetMext(CoordinateSystem.Global));
            DA.SetDataList(23, elements[0].Getfext(CoordinateSystem.Global));
            DA.SetDataList(24, elements[0].Getmext(CoordinateSystem.Global));

            // réactions
            DA.SetDataList(25, elements[0].GetFr(CoordinateSystem.Global));
            DA.SetDataList(26, elements[0].GetMr(CoordinateSystem.Global));

            DA.SetDataList(27, elements[0].M_g);
            DA.SetDataList(28, elements[0].M_h_l);
            DA.SetDataList(29, elements[0].M_h_r);

            DA.SetDataList(30, elements[0].Q);
            DA.SetDataList(31, elements[0].Q_l);
            DA.SetDataList(32, elements[0].Q_r);

            elements[0].GetShearForceAtNodes();
            DA.SetDataList(33, elements[0].V_M_g);
            DA.SetDataList(34, elements[0].V_M_h_l);
            DA.SetDataList(35, elements[0].V_M_h_r);

            DA.SetDataList(36, elements[0].N);
            DA.SetDataList(37, elements[0].N_l);
            DA.SetDataList(38, elements[0].N_r);

            DA.SetDataList(39, elements[0].Rx_axial);
            DA.SetDataList(40, elements[0].Rx_shear_M);
            DA.SetDataList(41, elements[0].Rx_shear_Q);
            DA.SetDataList(42, elements[0].Rθ_M);
            DA.SetDataList(43, elements[0].Rθ_Q);

            //// dynamic relaxation
            DA.SetDataList(44, Ec_x_history);
            DA.SetDataList(45, Ec_θ_history);
            DA.SetDataList(46, lm_x[0]);
            DA.SetDataList(47, lm_θ[0]);
            DA.SetDataList(48, v_x[0]);
            DA.SetDataList(49, v_θ[0]);
            //DA.SetDataList(33, R_x[0]);
            //DA.SetDataList(34, R_θ[0]);


            index++;
        }

        private void DR_Relax(int iteration_max)
        {
            Rhino.RhinoApp.WriteLine("RELAX");

            // TRACKING
            numPic_x = 0;
            numPic_θ = 0;
            numIteration_x = 0;
            numIteration_θ = 0;
            iterationHistory = new List<int>();
            chronoHistory = new List<double>();

            //Ec_x = 0; Ec_x_0 = -1; Ec_x_1 = -1;
            //Ec_θ = 0; Ec_θ_0 = -1; Ec_θ_1 = -1;

            Ec_x_lim = 1e-10;
            Ec_θ_lim = 1e-6;

            // INIT
            Rhino.RhinoApp.WriteLine("PB SETUP");

            elements = new CurvedBeam[N];
            elements[0] = new CurvedBeam(0, "", section_prop.ToArray(), material_prop, sections_0.ToArray(), sections_i.ToArray());

            // EXTERNAL FORCES
            //elements[0].Fext[elements[0].Nn - 1].Z = -100000;

            // MOMENTS (uncomment pour tester le moment d'extrémité)
            //elements[0].Mext[elements[0].Nn - 1] =  10e6;
            //elements[0].Update_mext();

            frame = new MFrame[N][];
            lm_x = new double[N][];
            lm_θ = new double[N][];
            R_x = new MVector[N][];
            R_θ = new double[N][];
            v_x = new MVector[N][];
            v_θ = new double[N][];

            for (int ei = 0; ei < N; ei++)
            {
                int n = elements[ei].Nn;
                frame[ei] = elements[ei].MaterialFrame;
                lm_x[ei] = new double[n];
                lm_θ[ei] = new double[n];
                R_x[ei] = elements[ei].Rx;
                R_θ[ei] = elements[ei].Rθ;
                v_x[ei] = new MVector[n];
                v_θ[ei] = new double[n];
            }

            double M1 = 1 * 50 * 1e4;
            double M2 = 1 * 20 * -1e4;
            double Q = 0 * 1e6;
            double Fz = -0e3;
            double Fn = 0 * 1e3;

            int nmid = elements[0].Nn / 2;
            int nend = elements[0].Nn - 1;
            //elements[0].Fext[ind] = 1e-2 * -1e6 * MVector.ZAxis;
            //elements[0].mext[ind-1] = new MVector(0, 1e-1*-M2, Q);

            // Example saut de courbure
            //var m = new MVector(0, 20e-1 * -M2, 0 * Q);
            //elements[0].mext[ind-1] = m;
            //elements[0].mext[ind] = m;
            //elements[0].mext[ind+1] = m;

            // Example avec Moment de Torsion
            var qext = 0 * 0.2 * 1e5;
            for (int i = 2; i < elements[0].Ne - 2; i++)
            {
               // elements[0].mext[i].Z = qext;
            }
            //elements[0].Mext[ind].Z = Q;

            // example shear linéique
            //elements[0].Fext[0].Z = 0 * Fz;

            //var f1ext = -1 * 1e3;
            //for (int i = 2; i < elements[0].Ne-2; i++)
            //{
            //    elements[0].fext[i].X = f1ext;
            //}
            //elements[0].Fext[ind].Z = 1*Fz;

            //elements[0].Fext[ind].X = 0*Fn;
            //elements[0].Mext[ind] = new MVector(M1, M2, Q);
            //elements[0].Mext[ind+2] = new MVector(0, 1e-1*M2, Q);

            //elements[0].Mext[nend] = new MVector(M1, M2, Q);
            //elements[0].Fext[elements[0].Nn - 1].Z = 0;


            //elements[0].Update_mext();
            //elements[0].Update_Fext();

            var constraints_list = new List<Constraint>();

            switch (bc_start)
            {
                case (int)BoundaryConditionType.Free:
                    elements[0].Mext[0] = new MVector(M1, M2, 0);
                    elements[0].Fext[0] = new MVector(0, 0, 0 * 1e6);

                    break;
                case (int)BoundaryConditionType.Clamped:
                    constraints_list.Add(BoundaryCondition.AddClampedBoundaryCondition(ref elements[0], Boundary.Start));
                    break;
                default:
                    constraints_list.Add(BoundaryCondition.AddPinnedBoundaryCondition(ref elements[0], Boundary.Start));
                    break;
            }

            switch (bc_end)
            {
                case (int)BoundaryConditionType.Free:
                    //elements[0].Mext[elements[0].Nn - 1] = new MVector(M1, M2,Q);
                    //elements[0].Fext[elements[0].Nn - 1] = new MVector(0, 0, 0*1e4);
                    break;
                case (int)BoundaryConditionType.Clamped:
                    constraints_list.Add(BoundaryCondition.AddClampedBoundaryCondition(ref elements[0], Boundary.End));
                    break;
                default:
                    constraints_list.Add(BoundaryCondition.AddPinnedBoundaryCondition(ref elements[0], Boundary.End));
                    break;
            }

            constraints = constraints_list.ToArray();

            foreach (var cst in constraints)
            {
                Rhino.RhinoApp.WriteLine(cst.ToString());
            }

            // applied displacements
            apd = new Constraint[0];
            //apd[0] = AppliedDisplacement.AddAppliedDisplacement(ref elements[0], 0, new MVector(0, 0, 0), CoordinateSystem.Material);
            //apd[0] = AppliedDisplacement.AddAppliedDisplacement(ref elements[0], elements[0].Nn - 1, Math.PI / 6, MaterialAxis.t);

            watch = new Stopwatch();
            watch.Start();
            // INIT
            Init();

            // RESET
            if (numIteration_x < iteration_max)
            {
                Reset_x();
                Reset_θ();
            }
            
            // RUN
            while (numIteration_x < iteration_max)
            {

                Run_x();

                // QUASISTATIC
                numIteration_θ = 0;
                //Reset_θ();
                while (numIteration_θ < 1)
                {
                    Run_θ();
                    //if (Ec_θ < Ec_θ_lim)
                    //{
                    //    //Rhino.RhinoApp.WriteLine("    Ec_θ[CVG | " + numPic_θ + " | " + numIteration_θ + "] = " + string.Format("{0:E2}", Ec_θ));
                    //    break;
                    //}
                }

                //Rhino.RhinoApp.WriteLine("    Ec_x[" + numIteration_x + " | " + numPic_x + "] = " + string.Format("{0:E2}", Ec_x));
                //Rhino.RhinoApp.WriteLine("    Ec_θ[" + numIteration_θ + " | " + numPic_θ + "] = " + string.Format("{0:E2}", Ec_θ));

                if (Ec_x < Ec_x_lim && Ec_θ < Ec_θ_lim)
                {
                    break;
                }
            }
            watch.Stop();

            //Enforce_x();
            //Enforce_t();
            //Enforce_θ();

            Rhino.RhinoApp.WriteLine("");

            if (numIteration_x == iteration_max)
            {
                Rhino.RhinoApp.WriteLine("Ec_x[MAX = " + numIteration_x + "] = " + string.Format("{0:E2}", Ec_x));
                Rhino.RhinoApp.WriteLine("Ec_θ[MAX = " + numIteration_θ + "] = " + string.Format("{0:E2}", Ec_θ));
            }
            else
            {
                Rhino.RhinoApp.WriteLine("Ec_x[CVG = " + numIteration_x + "] = " + string.Format("{0:E2}", Ec_x));
                Rhino.RhinoApp.WriteLine("Ec_θ[CVG = " + numIteration_θ + "] = " + string.Format("{0:E2}", Ec_θ));
            }

            double Ep = elements[0].Ea + elements[0].Eb + elements[0].Et;
            Rhino.RhinoApp.WriteLine("Ep = " + Ep);

        }


        // INIT
        private void Init()
        {
            // the model is set up in it's initial configuration
            // constraints are applied and resultant forces and moments are computed
            Rhino.RhinoApp.WriteLine("INIT_X : " + numIteration_x);

            // apply applied displacements
            foreach (var cst in apd) { cst.Init(); }

            // get boundary references (after applied displacements)
            foreach (var cst in constraints) { cst.Init(); }

            // update config
            UpdateDeformedConfig_x();
        }

        // TRANSLATION
        private void Reset_x()
        {
            //Rhino.RhinoApp.WriteLine("###### RESET X");
            Rhino.RhinoApp.WriteLine("Ec_x[" + numPic_x + "] = " + string.Format("{0:E2}", Ec_x));

            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetResultantNodalMoment();
                elements[ei].GetResultantNodalForce();
            }

            // LUMPED MASS
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].Update_lm_x(ref lm_x[ei]);
            }

            // VELOCITY
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    v_x[ei][nj] = 0.5 * (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_x = 0.0;
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    Ec_x += 0.5 * lm_x[ei][nj] * (v_x[ei][nj] * v_x[ei][nj]);
                }
            }
            Ec_x_history.Add(Ec_x);
            Ec_x_1 = Ec_x;
            Ec_x_0 = 0.0;

            // POSITION
            for (int ei = 0; ei < N; ei++)
            {
                MVector[] dx = new MVector[elements[ei].Nn];
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    dx[nj] = dt * v_x[ei][nj];
                }
                elements[ei].Move(dx);
                //elements[ei].GetDeformedConfig_x();
            }
            UpdateDeformedConfig_x();

            numIteration_x++;
        }
        private void Run_x()
        {
            //Rhino.RhinoApp.WriteLine("###### RUN X");
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetResultantNodalMoment();
                elements[ei].GetResultantNodalForce();
            }

            // VELOCITY
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    v_x[ei][nj] += (dt / lm_x[ei][nj]) * elements[ei].Rx[nj];
                }
            }

            // KINETIC ENERGY
            Ec_x = 0.0;
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
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
                for (int ei = 0; ei < N; ei++)
                {
                    MVector[] dx = new MVector[elements[ei].Nn];
                    for (int nj = 0; nj < elements[ei].Nn; nj++)
                    {
                        dx[nj] = dt * v_x[ei][nj];
                    }
                    elements[ei].Move(dx);
                    //elements[ei].GetDeformedConfig_x();
                }
                UpdateDeformedConfig_x();

                numIteration_x++;
            }
            else // KINETIC PIC REACHED
            {
                // INTERPOLATION & POSITION
                InterpolateEc_x(Ec_x_0, Ec_x_1, Ec_x);
                Ec_x_history[Ec_x_history.Count - 1] = Ec_x;

                numPic_x++;

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
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    v_x[ei][nj] -= (dt / lm_x[ei][nj]) * R_x[ei][nj];
                }
            }

            // COMPUTE BACKWARD DISPLACEMENT AND MOVE
            for (int ei = 0; ei < N; ei++)
            {
                MVector[] dx = new MVector[elements[ei].Nn];
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    dx[nj] = -q * dt * v_x[ei][nj];
                }
                elements[ei].Move(dx);
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
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetResultantNodalMoment();
                elements[ei].GetResultantNodalForce();
            }

            // LUMPED MASS
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].Update_lm_θ(ref lm_θ[ei]);
            }

            // VELOCITY
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    v_θ[ei][nj] = 0.5 * (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_θ = 0.0;
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    Ec_θ += 0.5 * lm_θ[ei][nj] * (v_θ[ei][nj] * v_θ[ei][nj]);
                }
            }
            Ec_θ_history.Add(Ec_θ);
            Ec_θ_1 = Ec_θ;
            Ec_θ_0 = 0;

            // POSITION
            for (int ei = 0; ei < N; ei++)
            {
                double[] dθ = new double[elements[ei].Nn];
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    dθ[nj] = dt * v_θ[ei][nj];
                    //Rhino.RhinoApp.WriteLine("dθ[" + nj + "] = " + dθ[nj]);
                }
                elements[ei].Move(dθ);
                //elements[ei].GetDeformedConfig_θ();
            }
            UpdateDeformedConfig_θ();

            numIteration_θ++;
        }
        private void Run_θ()
        {
            //Rhino.RhinoApp.WriteLine("###### RUN θ");
            // UPDATE RESULTANT NODAL FORCE AND MOMENT
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetResultantNodalMoment();
                elements[ei].GetResultantNodalForce();
            }

            // VELOCITY
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    v_θ[ei][nj] += (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // KINETIC ENERGY
            Ec_θ = 0.0;
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
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
                for (int ei = 0; ei < N; ei++)
                {
                    double[] dθ = new double[elements[ei].Nn];
                    for (int nj = 0; nj < elements[ei].Nn; nj++)
                    {
                        dθ[nj] = dt * v_θ[ei][nj];
                    }
                    elements[ei].Move(dθ);
                    //elements[ei].GetDeformedConfig_θ();
                }
                UpdateDeformedConfig_θ();

                numIteration_θ++;
            }
            else // KINETIC PIC REACHED
            {
                // INTERPOLATION
                InterpolateEc_θ(Ec_θ_0, Ec_θ_1, Ec_θ);
                Ec_θ_history[Ec_θ_history.Count - 1] = Ec_θ;

                numPic_θ++;

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
            for (int ei = 0; ei < N; ei++)
            {
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    v_θ[ei][nj] -= (dt / lm_θ[ei][nj]) * R_θ[ei][nj];
                }
            }

            // COMPUTE BACKWARD DISPLACEMENT
            for (int ei = 0; ei < N; ei++)
            {
                double[] dθ = new double[elements[ei].Nn];
                for (int nj = 0; nj < elements[ei].Nn; nj++)
                {
                    dθ[nj] = -q * dt * v_θ[ei][nj];
                }
                elements[ei].Move(dθ);
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
            for (int ei = 0; ei < N; ei++) { elements[ei].GetCenterlineProperties(); }

            // enforce tangent constraints (t, κb)
            foreach (var cst in constraints) { cst.Enforce_Mr(); }

            // update curvature binormal (κb)
            for (int ei = 0; ei < N; ei++)
            { elements[ei].GetCurvatureBinormal(); }

            // update material frames
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].UpdateMaterialFrame();
                //elements[ei].GetMidEdgeQuantities();
            }

            // update internal bending and twisting moments
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetBendingMoment();
                elements[ei].GetTwistingMoment();
            }

            // update internal axial and shear forces
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetAxialForce();
                elements[ei].GetShearForce();
            }

            // update resulting internal nodal force and internal nodal moment
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetInternalNodalMoment();
                elements[ei].GetInternalNodalForce();
            }

            // enforce nodal force and moment contraints
            foreach (var cst in constraints) { cst.Enforce_Qr(); }
            foreach (var cst in constraints) { cst.Enforce_Fr(); }
        }
        private void UpdateDeformedConfig_θ()
        {
            // update internal bending and twisting moments
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetBendingMoment();
                elements[ei].GetTwistingMoment();
            }

            // update internal shear forces
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetShearForce();
            }

            // update resulting internal nodal force and internal nodal moment
            for (int ei = 0; ei < N; ei++)
            {
                elements[ei].GetInternalNodalMoment();
                elements[ei].GetInternalNodalForce();
            }

            // enforce nodal force and moment contraints
            foreach (var cst in constraints) { cst.Enforce_Qr(); }
            foreach (var cst in constraints) { cst.Enforce_Fr(); }
        }


    }



}
