using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.CoreLib3;
using System.Diagnostics;
using Rhino;
<<<<<<< HEAD

namespace TMarsupilami.Gh.Component
{
    public class Comp_BenchSingleBeam : GH_Component
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
        private List<Section> sections;
        private List<Material> materials;
        private Beam[] elements;
        private List<Support> bc_list;
        private Beam_4DOF_D beam;
        private KDRSolver solver;
        private Stopwatch watch;

        double Ec_x_lim;
        double Ec_θ_lim;

        MVector F, M, f, m;


        // CONSTRUCTOR
        public Comp_BenchSingleBeam()
            : base("Bench 1B", "Bench 1D", "Test cases for benchmarking", "TMarsupilami", "Bench")
        {
            frames_0 = new List<MFrame>();
            frames_i = new List<MFrame>();
            N = 1;
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{9941BCC8-326A-405D-80CD-0B604A9D23C1}"); }
=======
using TMarsupilami.BenchProblem;

namespace TMarsupilami.Gh.Component
{
    public class Comp_ReadSingleBeamProblem : GH_Component
    {

        // CONSTRUCTOR
        public Comp_ReadSingleBeamProblem()
            : base("Read Single Beam Problem", "Read SBP", "Read a SingleBeamProblem from a json file", "TMarsupilami", "Bench")
        {
        } 
        public override Guid ComponentGuid
        {
            get { return new Guid("{40D48E49-78CC-4051-93CC-B70E3A025235}"); }
>>>>>>> Origin/NewShearThesis
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        // PARAMETERS
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
<<<<<<< HEAD
            pManager.AddParameter(new Param_MFrame(), "Rest position", "sections_0", "Beam's sections in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Initial position", "sections_i", "Beam's sections in initial position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);

            pManager.AddNumberParameter("Section b1", "b1", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("Section b2", "b2", "", GH_ParamAccess.list);

            pManager.AddIntegerParameter("Boundary Condition at Start", "bc_start", "(FREE = 0, PINNED = 1, CLAMPED = 2)", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("Boundary Condition at End", "bc_end", "(FREE = 0, PINNED = 1, CLAMPED  =2)", GH_ParamAccess.item, 1);

            pManager.AddParameter(new Param_MVector(), "Concentrated Force", "F", "Concentrated force at mid span.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Concentrated Moment", "M", "Concentrated moment at mid span.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Distributed Force", "f", "Distributed force applied to [0,L/2].", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MVector(), "Distributed Moment", "m", "Distributed moment applied to [0,L/2].", GH_ParamAccess.item);

            pManager.AddIntegerParameter("iteration_max", "N_max", "Total number of iterations to run", GH_ParamAccess.item, 10);
            pManager.AddBooleanParameter("reset", "reset", "Reset the engine", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("refine", "refine", "Press button to restart the solver with a refined element.", GH_ParamAccess.item, false);

            pManager[6].Optional = true;
            pManager[7].Optional = true;
            pManager[8].Optional = true;
            pManager[9].Optional = true;

            pManager[12].Optional = true;


        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.RegisterParam(new Param_MBeam(), "Beams", "B", "Beam elements.");
=======
            pManager.AddTextParameter("File Path", "path", "The path to the json file", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.RegisterParam(new Param_MBeam(), "Beams", "B", "Beam element", GH_ParamAccess.item);
>>>>>>> Origin/NewShearThesis
        }

        // SOLVER
        protected override void SolveInstance(IGH_DataAccess DA)
        {
<<<<<<< HEAD
            frames_0.Clear();
            frames_i.Clear();

            b1 = new List<double>();
            b2 = new List<double>();

            F = MVector.Zero;
            M = MVector.Zero;
            f = MVector.Zero;
            m = MVector.Zero;

            bool isRefine = false;

            if (!DA.GetDataList(0, frames_0)) { return; }
            if (!DA.GetDataList(1, frames_i)) { return; }

            if (!DA.GetDataList(2, b1)) { return; }
            if (!DA.GetDataList(3, b2)) { return; }

            if (!DA.GetData(4, ref bc_start)) { return; }
            if (!DA.GetData(5, ref bc_end)) { return; }

            if (!DA.GetData(6, ref F)) { F = MVector.Zero; }
            if (!DA.GetData(7, ref M)) { M = MVector.Zero; }
            if (!DA.GetData(8, ref f)) { f = MVector.Zero; }
            if (!DA.GetData(9, ref m)) { m = MVector.Zero; }

            if (!DA.GetData(10, ref iteration_max)) { return; }
            if (!DA.GetData(11, ref loop_reset)) { return; }

            DA.GetData(12, ref isRefine);

            Ec_x_lim = 1e-10;
            Ec_θ_lim = 1e-6;

            if (isRefine)
            {
                beam.Refine();

                solver = new KDRSolver(elements, bc_list, new List<Link>(), iteration_max, Ec_x_lim, Ec_θ_lim);
                solver.OnEnergyPeak_x += OnKineticEnergyPeak_x;
                solver.OnConvergence += OnConvergence;
                solver.OnNotConvergence += OnNotConvergence;

                watch = new Stopwatch();
                watch.Start();
                solver.Run(iteration_max);
                watch.Stop();
            }
            else
            {
                if (loop_reset == true) // Premier Calcul
                {
                    loop_reset_cache = loop_reset;

                    int n = frames_i.Count;
                    sections = new List<Section>();

                    // n-1 section definitions
                    if (n == 1)
                    {
                        var sprop = Section.RectangularSection(b1[0], b2[0]);
                        for (int i = 0; i < n - 1; i++)
                        {
                            sections.Add(sprop);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < b1.Count; i++)
                        {
                            sections.Add(Section.RectangularSection(b1[i], b2[i]));
                        }
                    }

                    materials = new List<Material>() { new Material(StandardMaterials.GFRP) };
                    DR_Relax(iteration_max);
                }

                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms");
                this.Message = "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms";

                DA.SetDataList(0, solver.elements_x);
            }           
        }

        private void DR_Relax(int iteration_max)
        {          
            // INIT
            beam = new Beam_4DOF_D(frames_0, frames_i, sections, materials);
            elements = new Beam[1] { beam };

            bc_list = new List<Support>();
            switch (bc_start)
=======
            string path = "";
            
            if (!DA.GetData(0, ref path)) { return; }

            var pb = SingleBeamProblem.DeSerialize(path);


            var Ec_x_lim = 1e-10;
            var Ec_θ_lim = 1e-6;

            int n = pb.RestConfiguration.Count;
            var sections = new List<Section>();
            var materials = new List<Material>();

            for (int i = 0; i < n - 1; i++)
            {
                sections.Add(Section.RectangularSection(pb.b1, pb.b2));
            }

            materials.Add(new Material(StandardMaterials.GFRP));

            // INIT
            var beam = new Beam_4DOF_D(pb.RestConfiguration, pb.ActualConfiguration, sections, materials);
            var elements = new Beam[1] { beam };

            var bc_list = new List<Support>();
            switch (pb.Start)
>>>>>>> Origin/NewShearThesis
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
<<<<<<< HEAD
            switch (bc_end)
            {                            
=======
            switch (pb.End)
            {
>>>>>>> Origin/NewShearThesis
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[0], Boundary.End);
                    break;
                default:
                    Support.AddPinnedSupport(elements[0], Boundary.End);
                    break;
            }


<<<<<<< HEAD

            int nh_mid = elements[0].Nvh / 2;

            var loads = new List<BeamVectorLoad>();
            loads.Add(BeamVectorLoad.Create_Fext(F, nh_mid, beam, true));
            loads.Add(BeamVectorLoad.Create_Mext(M, nh_mid, beam, false));

            int ng_mid = elements[0].Nvg / 2;
            for (int i = 0; i < ng_mid; i++)
            {
                loads.Add(BeamVectorLoad.Create_fext(f, i, beam, true));
                loads.Add(BeamVectorLoad.Create_mext(m, i, beam, false));
            }

            beam.Load(loads);

            solver = new KDRSolver(elements, bc_list, new List<Link>(), iteration_max,  Ec_x_lim, Ec_θ_lim);
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
=======
            // Add Some Loadds
            int nh_mid = elements[0].Nvh / 2;
            var loads = new List<BeamVectorLoad>();
            loads.Add(BeamVectorLoad.Create_Fext(new MVector(0, 0, 100), nh_mid, beam, true));

            beam.Load(loads);

            var solver = new KDRSolver(elements, bc_list, new List<Link>(), 1, Ec_x_lim, Ec_θ_lim);
            solver.OnEnergyPeak_x += Nothing;
            solver.OnConvergence += Nothing;
            solver.OnNotConvergence += Nothing;

            DA.SetData(0, beam);        
        }

        private static void Nothing(KDRSolver solver)
        {
>>>>>>> Origin/NewShearThesis
        }
    }



}
