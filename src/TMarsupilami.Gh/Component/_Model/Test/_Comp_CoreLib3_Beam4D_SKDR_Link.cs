using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.CoreLib3;
using System.Diagnostics;
using Rhino;

namespace TMarsupilami.Gh.Component
{
    public class _Comp_CoreLib3_Beam4D_SKDR_Link : GH_Component
    {
        private bool loop_reset = true;
        private bool loop_reset_cache = true;
        private int iteration_max;
        private bool is_base = true;

        // RELAX
        int N;
        private int index;

        private List<double> b1, h1, b2, h2;
        private List<MFrame> f1_0, f1_i, f2_0, f2_i;
        private int bc1_start, bc1_end, bc2_start, bc2_end;
        private List<Section> sections_1, sections_2;
        private List<Material> materials;
        private Beam[] elements;
        private KDRSolver solver;
        private Stopwatch watch;

        // CONSTRUCTOR
        public _Comp_CoreLib3_Beam4D_SKDR_Link()
            : base("4_DOF_DISC_SKDR_Link", "4_DOF_DISC_SKDR", "Double simple beam with various boundary conditions", "TMarsupilami", "Core3Lib")
        {
            f1_0 = new List<MFrame>();
            f1_i = new List<MFrame>();
            f2_0 = new List<MFrame>();
            f2_i = new List<MFrame>();
            N = 1;
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{AF0E6B99-B3C0-4090-964E-3E6F2D1C01B9}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        // PARAMETERS
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Rest position", "frames_0", "Beam's sections in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Initial position", "frames_i", "Beam's sections in initial position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);

            pManager.AddNumberParameter("Section b1", "b1", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Section b2", "b2", "", GH_ParamAccess.list);

            pManager.AddIntegerParameter("Boundary Condition at Start", "bc_start", "(FREE = 0, PINNED = 1, CLAMPED = 2)", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("Boundary Condition at End", "bc_end", "(FREE = 0, PINNED = 1, CLAMPED  =2)", GH_ParamAccess.item, 1);

            pManager.AddParameter(new Param_MFrame(), "Rest position", "frames_0", "Beam's sections in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Initial position", "frames_i", "Beam's sections in initial position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);

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
            pManager.RegisterParam(new Param_MBeam(), "Beams", "B", "Beam elements.");
        }

        // SOLVER
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            f1_0.Clear();
            f1_i.Clear();
            f2_0.Clear();
            f2_i.Clear();

            b1 = new List<double>();
            h1 = new List<double>();
            b2 = new List<double>();
            h2 = new List<double>();

            if (!DA.GetDataList(0, f1_0)) { return; }
            if (!DA.GetDataList(1, f1_i)) { return; }

            if (!DA.GetDataList(2, b1)) { return; }
            if (!DA.GetDataList(3, h1)) { return; }

            if (!DA.GetData(4, ref bc1_start)) { return; }
            if (!DA.GetData(5, ref bc1_end)) { return; }

            if (!DA.GetDataList(6, f2_0)) { return; }
            if (!DA.GetDataList(7, f2_i)) { return; }

            if (!DA.GetDataList(8, b2)) { return; }
            if (!DA.GetDataList(9, h2)) { return; }

            if (!DA.GetData(10, ref bc2_start)) { return; }
            if (!DA.GetData(11, ref bc2_end)) { return; }

            if (!DA.GetData(12, ref iteration_max)) { return; }
            if (!DA.GetData(13, ref loop_reset)) { return; }

            if (!DA.GetData(14, ref is_base)) { return; }


            if (loop_reset == true) // Premier Calcul
            {
                loop_reset_cache = loop_reset;
                DR_Relax(iteration_max);
            }

            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms");
            this.Message = "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms";

            DA.SetDataList(0, solver.elements_x);
        }

        private void DR_Relax(int iteration_max)
        {
            var Ec_x_lim = 1e-10;
            var Ec_θ_lim = 1e-6;

            // INIT
            var bc_list = new List<Support>();

            materials = new List<Material>() { new Material(StandardMaterials.GFRP) };


            // BEAM 1
            sections_1 = new List<Section>();
            for (int i = 0; i < b1.Count; i++)
            {
                sections_1.Add(Section.RectangularSection(b1[i], h1[i]));
            }
            Beam_4DOF_D beam1 = new Beam_4DOF_D(f1_0, f1_i, sections_1, materials);
            
            // BEAM 2
            sections_2 = new List<Section>();
            for (int i = 0; i < b2.Count; i++)
            {
                sections_2.Add(Section.RectangularSection(b2[i], h2[i]));
            }
            Beam_4DOF_D beam2 = new Beam_4DOF_D(f2_0, f2_i, sections_2, materials);

            // LOADING
            var loads_1 = new List<BeamVectorLoad>();
            var loads_2 = new List<BeamVectorLoad>();
            beam2.Load(loads_2);
            beam1.Load(loads_1);

            // ELEMENTS
            elements = new Beam[2] { beam1, beam2 };

            // BC
            switch (bc1_start)
            {
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[0], Boundary.Start);
                    break;
                default:
                    Support.AddPinnedSupport(elements[0], Boundary.Start);
                    break;
            }
            switch (bc1_end)
            {
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[0], Boundary.End);
                    break;
                default:
                    Support.AddPinnedSupport(elements[0], Boundary.End);
                    break;
            }
            switch (bc2_start)
            {
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[1], Boundary.Start);
                    break;
                default:
                    Support.AddPinnedSupport(elements[1], Boundary.Start);
                    break;
            }
            switch (bc2_end)
            {
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[1], Boundary.End);
                    break;
                default:
                    Support.AddPinnedSupport(elements[1], Boundary.End);
                    break;
            }

            // LINK
            int n1 = f1_0.Count / 2;
            int n2 = f2_0.Count / 2;
            Link link;
            List<Link> links = new List<Link>();

            //links.Add(Link.CreateElasticPinnedLink(beam1, n1, beam2, n2, 1e10));
            links.Add(Link.CreateElasticSwivelLink(beam1, n1, beam2, n2, 1e10, 1e8));



            // SOLVER
            solver = new KDRSolver(elements, bc_list, links, iteration_max, Ec_x_lim, Ec_θ_lim);
            solver.OnEnergyPeak_x += OnKineticEnergyPeak_x;
            solver.OnConvergence += OnConvergence;
            solver.OnNotConvergence += OnNotConvergence;

            watch = new Stopwatch();
            watch.Start();
            solver.Run(iteration_max);
            watch.Stop();
        }
       
        private static void OnKineticEnergyPeak_x(KDRSolver solver)
        {
            RhinoApp.WriteLine("EC_x[" + solver.NumberOfKineticPeaks_x + "] = " + string.Format("{0:E2}", solver.Ec_x));
        }

        private static void OnConvergence(KDRSolver solver)
        {
            Rhino.RhinoApp.WriteLine("Ec_x[CVG = " + solver.CurrentIteration_x + "] = " + string.Format("{0:E8}", solver.Ec_x));
            Rhino.RhinoApp.WriteLine("Ec_θ[CVG = " + solver.CurrentIteration_θ + "] = " + string.Format("{0:E8}", solver.Ec_θ));
        }

        private static void OnNotConvergence(KDRSolver solver)
        {
            Rhino.RhinoApp.WriteLine("Ec_x[ENDED = " + solver.CurrentIteration_x + "] = " + string.Format("{0:E8}", solver.Ec_x));
            Rhino.RhinoApp.WriteLine("Ec_θ[ENDED = " + solver.CurrentIteration_θ + "] = " + string.Format("{0:E8}", solver.Ec_θ));
        }

    }



}
