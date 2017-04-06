using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.CoreLib2;
using System.Diagnostics;
using Rhino;

namespace TMarsupilami.Gh.Component
{
    public class _Comp_CoreLib2_Beam4D_SKDR2 : GH_Component
    {
        private bool loop_reset = true;
        private bool loop_reset_cache = true;
        private int iteration_max;
        private bool is_base = true;

        // RELAX
        int N;
        private int index;

        private List<double> b1, b2;
        private List<MFrame> frames_0, frames_i;
        private int bc_start, bc_end;
        private List<SectionProperty> sections;
        private List<MaterialProperty> materials;
        private IDRElement[] elements;
        private KDRSolver2 solver;
        private Stopwatch watch;
        private Model model;


        // CONSTRUCTOR
        public _Comp_CoreLib2_Beam4D_SKDR2()
            : base("4_DOF_DISC_SKDR2", "4_DOF_DISC_SKDR2", "Single simple beam with various boundary conditions", "TMarsupilami", "Core2Lib")
        {
            frames_0 = new List<MFrame>();
            frames_i = new List<MFrame>();
            N = 1;
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{DBDA9BA0-ADFF-4A00-8893-53BF1040588C}"); }
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
            frames_0.Clear();
            frames_i.Clear();

            b1 = new List<double>();
            b2 = new List<double>();

            if (!DA.GetDataList(0, frames_0)) { return; }
            if (!DA.GetDataList(1, frames_i)) { return; }

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

                int n = frames_i.Count;
                sections = new List<SectionProperty>();

                // n-1 section definitions
                if (n == 1)
                {
                    var sprop = SectionProperty.RectangularSection(b1[0], b2[0]);
                    for (int i = 0; i < n - 1; i++)
                    {
                        sections.Add(sprop);
                    }
                }
                else
                {
                    for (int i = 0; i < n - 1; i++)
                    {
                        sections.Add(SectionProperty.RectangularSection(b1[i], b2[i]));
                    }
                }

                materials = new List<MaterialProperty>() { new MaterialProperty(StandardMaterials.GFRP) };
                DR_Relax(iteration_max);
            }


            DA.SetData(0, "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms");
            DA.SetData(1, solver.CurrentIteration_x);

            // rest configuration
            //DA.SetDataList(2, elements[0].l_0);
            //DA.SetDataList(3, elements[0].κb_0);
            //DA.SetDataList(4, elements[0].κ1_0);
            //DA.SetDataList(5, elements[0].κ2_0);
            //DA.SetDataList(6, elements[0].twist_0);
            //DA.SetDataList(7, elements[0].τ_0);

            // actual configuration
            DA.SetDataList(8, solver.elements_x[0].MaterialFrames);
            //DA.SetDataList(9, elements[0].e);
            //DA.SetDataList(10, elements[0].t);
            //DA.SetDataList(11, elements[0].t_g);
            //DA.SetDataList(12, elements[0].t_h_l);
            //DA.SetDataList(13, elements[0].t_h_r);

            //DA.SetDataList(14, elements[0].l);
            //DA.SetDataList(15, elements[0].ε);

            //DA.SetDataList(16, elements[0].κb_g);
            //DA.SetDataList(17, elements[0].κb_h_l);
            //DA.SetDataList(18, elements[0].κb_h_r);
            //DA.SetDataList(19, elements[0].twist);
            //DA.SetDataList(20, elements[0].τ);

            //DA.SetDataList(21, elements[0].GetFext(CoordinateSystem.Global));
            //DA.SetDataList(22, elements[0].GetMext(CoordinateSystem.Global));
            //DA.SetDataList(23, elements[0].Getfext(CoordinateSystem.Global));
            //DA.SetDataList(24, elements[0].Getmext(CoordinateSystem.Global));

            // réactions
            //DA.SetDataList(25, elements[0].GetFr(CoordinateSystem.Global));
            //DA.SetDataList(26, elements[0].GetMr(CoordinateSystem.Global));

            //DA.SetDataList(27, elements[0].M_g);
            //DA.SetDataList(28, elements[0].M_h_l);
            //DA.SetDataList(29, elements[0].M_h_r);

            //DA.SetDataList(30, elements[0].Q);
            //DA.SetDataList(31, elements[0].Q_l);
            //DA.SetDataList(32, elements[0].Q_r);

            //elements[0].GetShearForceAtNodes();
            //DA.SetDataList(33, elements[0].V_M_g);
            //DA.SetDataList(34, elements[0].V_M_h_l);
            //DA.SetDataList(35, elements[0].V_M_h_r);

            //DA.SetDataList(36, elements[0].N);
            //DA.SetDataList(37, elements[0].N_l);
            //DA.SetDataList(38, elements[0].N_r);

            //DA.SetDataList(39, elements[0].Rx_axial);
            //DA.SetDataList(40, elements[0].Rx_shear_M);
            //DA.SetDataList(41, elements[0].Rx_shear_Q);
            //DA.SetDataList(42, elements[0].Rθ_M);
            //DA.SetDataList(43, elements[0].Rθ_Q);

            //// dynamic relaxation
            //DA.SetDataList(44, Ec_x_history);
            //DA.SetDataList(45, Ec_θ_history);
            //DA.SetDataList(46, lm_x[0]);
            //DA.SetDataList(47, lm_θ[0]);
            //DA.SetDataList(48, v_x[0]);
            //DA.SetDataList(49, v_θ[0]);


            index++;
        }

        private void DR_Relax(int iteration_max)
        {
            var Ec_x_lim = 1e-10;
            var Ec_θ_lim = 1e-6;

            // INIT
            Rhino.RhinoApp.WriteLine("PB SETUP");
            BeamLayout layout = new BeamLayout(1, frames_0, frames_i, sections, materials, false);
            elements = new IDRElement[1] { new Beam4D_2(layout) };

            model = new Model();
            int beamId = model.AddBeamLayout(frames_0, frames_i, sections, materials, false);

            // EXTERNAL FORCES
            //elements[0].Fext[elements[0].Nn - 1].Z = -100000;

            // MOMENTS (uncomment pour tester le moment d'extrémité)
            //elements[0].Mext[elements[0].Nn - 1] =  10e6;
            //elements[0].Update_mext();

            double M1 = 1 * 50 * 1e4;
            double M2 = 1 * 20 * -1e4;
            double Q = 0 * 1e6;
            double Fz = -0e3;
            double Fn = 0 * 1e3;

            int nmid = elements[0].Nv / 2;
            int nend = elements[0].Nv - 1;
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
                //elements[0].mext_g[i].Z = qext;
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

            var bc_list = new List<IDRConstraint>();
            var bc_ids = new List<int>();
            switch (bc_start)
            {
                case (int)BoundaryConditionType.Free:
                    elements[0].Mext_m[0] = new MVector(M1, M2, 0);
                    elements[0].Fext_g[0] = new MVector(0, 0, 0 * 1e6);
                    break;

                case (int)BoundaryConditionType.Clamped:
                    bc_list.Add(BoundaryCondition.AddClampedBoundaryCondition(elements[0], Boundary.Start));
                    bc_ids.Add(model.AddBoundaryConstraint(beamId, 0, BoundaryConditionType.Clamped));
                    break;

                default:
                    bc_list.Add(BoundaryCondition.AddPinnedBoundaryCondition(elements[0], Boundary.Start));
                    bc_ids.Add(model.AddBoundaryConstraint(beamId, 0, BoundaryConditionType.Pinned));
                    break;
            }

            switch (bc_end)
            {
                case (int)BoundaryConditionType.Free:
                    //elements[0].Mext[elements[0].Nn - 1] = new MVector(M1, M2,Q);
                    //elements[0].Fext[elements[0].Nn - 1] = new MVector(0, 0, 0*1e4);
                    break;
                case (int)BoundaryConditionType.Clamped:
                    bc_list.Add(BoundaryCondition.AddClampedBoundaryCondition(elements[0], Boundary.End));
                    bc_ids.Add(model.AddBoundaryConstraint(beamId, elements[0].Nv - 1, BoundaryConditionType.Clamped));
                    break;
                default:
                    bc_list.Add(BoundaryCondition.AddPinnedBoundaryCondition(elements[0], Boundary.End));
                    bc_ids.Add(model.AddBoundaryConstraint(beamId, elements[0].Nv-1, BoundaryConditionType.Pinned));
                    break;
            }

            //solver = new KDRSolver2(elements, bc_list, iteration_max, Ec_x_lim, Ec_θ_lim);
            solver = new KDRSolver2(model, iteration_max, Ec_x_lim, Ec_θ_lim);
            solver.OnEnergyPeak_x += OnKineticEnergyPeak_x;
            solver.OnConvergence += OnConvergence;

            watch = new Stopwatch();
            watch.Start();
            solver.Run(iteration_max);
            watch.Stop();
        }
       
        private static void OnKineticEnergyPeak_x(KDRSolver2 solver)
        {
            RhinoApp.WriteLine("EC_x[" + solver.NumberOfKineticPeaks_x + "] = " + string.Format("{0:E2}", solver.Ec_x));
        }

        private static void OnConvergence(KDRSolver2 solver)
        {
            Rhino.RhinoApp.WriteLine("Ec_x[CVG = " + solver.CurrentIteration_x + "] = " + string.Format("{0:E8}", solver.Ec_x));
            Rhino.RhinoApp.WriteLine("Ec_θ[CVG = " + solver.CurrentIteration_θ + "] = " + string.Format("{0:E8}", solver.Ec_θ));
        }

    }



}
