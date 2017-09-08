using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.CoreLib3;
using System.Diagnostics;
using Rhino;
using TMarsupilami.Gh.Type;
using Grasshopper.Kernel.Data;

namespace TMarsupilami.Gh.Component
{
    public class Comp_BenchSingleBeam : GH_Component
    {
        private bool loop_reset = true;
        private bool loop_reset_cache = true;
        private int iteration_max;

        // RELAX

        int nH, nV;
        KDRSolver solver;
        Stopwatch watch;

        // CONSTRUCTOR
        public Comp_BenchSingleBeam()
            : base("Bench Grid 2D", "Bench Grid 2D", "Test case for benchmarking", "TMarsupilami", "Bench")
        {
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{305ACEBD-FACE-4CAB-9446-C4479F9E467F}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        // PARAMETERS
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Rest position (H)", "Hr", "Beam's frames in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.tree);
            pManager.AddParameter(new Param_MFrame(), "Initial position (H)", "Hi", "Beam's frames in initial position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.tree);
            pManager.AddParameter(new Param_MFrame(), "Rest position (V)", "Vr", "Beam's frames in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.tree);
            pManager.AddParameter(new Param_MFrame(), "Initial position (V)", "Vi", "Beam's frames in initial position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.tree);

            pManager.AddNumberParameter("Section b1", "b1", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("Section b2", "b2", "", GH_ParamAccess.item);

          
            pManager.AddIntegerParameter("iteration_max", "N_max", "Total number of iterations to run", GH_ParamAccess.item, 10);
            pManager.AddBooleanParameter("reset", "reset", "Reset the engine", GH_ParamAccess.item, false);

        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.RegisterParam(new Param_MBeam(), "Beams", "B", "Beam elements.", GH_ParamAccess.list);
        }

        // SOLVER
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var Hr = new GH_Structure<GH_MFrame>();
            var Hi = new GH_Structure<GH_MFrame>();
            var Vr = new GH_Structure<GH_MFrame>();
            var Vi = new GH_Structure<GH_MFrame>();

            double b1 = 0;
            double b2 = 0;

            int bc_start = 1;
            int bc_end = 1;

            if (!DA.GetDataTree<GH_MFrame>(0, out Hr)) { return; }
            if (!DA.GetDataTree<GH_MFrame>(1, out Hi)) { return; }
            if (!DA.GetDataTree<GH_MFrame>(2, out Vr)) { return; }
            if (!DA.GetDataTree<GH_MFrame>(3, out Vi)) { return; }

            if (!DA.GetData(4, ref b1)) { return; }
            if (!DA.GetData(5, ref b2)) { return; }

            if (!DA.GetData(6, ref iteration_max)) { return; }
            if (!DA.GetData(7, ref loop_reset)) { return; }

            var Ec_x_lim = 1e-10;
            var Ec_θ_lim = 1e-6;

            var section = Section.RectangularSection(b1, b2);
            var material = new Material(StandardMaterials.GFRP);


            if (loop_reset == true) // Premier Calcul
            {
                loop_reset_cache = loop_reset;

                var elements = new List<Beam>();
                var supports = new List<Support>();
                var links = new List<Link>();

                // Creation H beams

                var Hbeams = LoadBeamElements(Hr, Hi, section, material, bc_start, bc_end);
                var Vbeams = LoadBeamElements(Vr, Vi, section, material, bc_start, bc_end);

                Hbeams = new Beam[] { Hbeams[Hbeams.Length / 2-1], Hbeams[Hbeams.Length / 2], Hbeams[Hbeams.Length / 2 + 1] };
                Vbeams = new Beam[] { Vbeams[Vbeams.Length / 2-1], Vbeams[Vbeams.Length / 2] , Vbeams[Vbeams.Length / 2 + 1] };

                int nH = Hbeams.Length;
                int nV = Vbeams.Length;

                double K = 1e7;
                double C = 4e6;

                for (int iH = 0; iH < nH; iH++)
                {
                    var bH = Hbeams[iH];

                    for (int jH = 0; jH < bH.Nv; jH++)
                    {
                        var mfH = bH.RestConfiguration[jH];

                        for (int iV = 0; iV < nV; iV++)
                        {
                            var bV = Vbeams[iV];

                            for (int jV = 0; jV < bV.Nv; jV++)
                            {
                                var mfV = bV.RestConfiguration[jV];

                                if (MPoint.DistanceTo(mfH.Origin, mfV.Origin) < 0.1)
                                {
                                    int index_H = bH.GlobalToHandleVertexIndex(jH);
                                    int index_V = bV.GlobalToHandleVertexIndex(jV);
                                    //links.Add(Link.CreateElasticPinnedLink(bH, index_H, bV, index_V, K));
                                    links.Add(Link.CreateElasticSwivelLink(bH, index_H, bV, index_V, K, C));

                                }
                            }

                        }
                    }

                }
                 

                elements.AddRange(Hbeams);
                elements.AddRange(Vbeams);
                
                solver = new KDRSolver(elements, new List<Support>(), links, iteration_max, Ec_x_lim, Ec_θ_lim);
                solver.OnEnergyPeak_x += OnKineticEnergyPeak_x;
                solver.OnConvergence += OnConvergence;
                solver.OnNotConvergence += OnNotConvergence;

                watch = new Stopwatch();
                watch.Start();
                solver.Run(iteration_max);
                watch.Stop();
            }

            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms");
            this.Message = "Elapsed = " + watch.Elapsed.TotalMilliseconds + " ms";

            DA.SetDataList(0, solver.elements_θ);
        }

       
        private Beam[] LoadBeamElements(GH_Structure<GH_MFrame> Fr, GH_Structure<GH_MFrame> Fi, Section section, Material material, int bc_start, int bc_end)
        {
            int nb = Fr.Branches.Count;
            var beams = new Beam[nb];

            for (int i = 0; i < nb; i++)
            {
                int nv = Fr.Branches[i].Count;
                var Xr = new MFrame[nv];
                var Xi = new MFrame[nv];
                var sections = new List<Section>();
                var materials = new List<Material>() { material };

                for (int j = 0; j < nv; j++)
                {
                    Xr[j] = Fr.Branches[i][j].Value;
                    Xi[j] = Fi.Branches[i][j].Value;
                }

                for (int j = 0; j < nv - 1; j++)
                {
                    sections.Add(section);
                }

                var beam = new Beam_4DOF_D(Xr, Xi, sections, materials);
                beams[i] = beam;

                switch (bc_start)
                {
                    case (int)SupportCondition.Free:
                        break;
                    case (int)SupportCondition.Clamped:
                        Support.AddClampedSupport(beam, Boundary.Start);
                        break;
                    default:
                        Support.AddPinnedSupport(beam, Boundary.Start);
                        break;
                }

                switch (bc_end)
                {
                    case (int)SupportCondition.Free:
                        break;
                    case (int)SupportCondition.Clamped:
                        Support.AddClampedSupport(beam, Boundary.End);
                        break;
                    default:
                        Support.AddPinnedSupport(beam, Boundary.End);
                        break;
                }

            }
            return beams;
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
