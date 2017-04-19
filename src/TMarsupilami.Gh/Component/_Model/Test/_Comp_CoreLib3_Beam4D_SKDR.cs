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
    public class _Comp_CoreLib3_Beam4D_SKDR : GH_Component
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
        private KDRSolver solver;
        private Stopwatch watch;

        // CONSTRUCTOR
        public _Comp_CoreLib3_Beam4D_SKDR()
            : base("4_DOF_DISC_SKDR", "4_DOF_DISC_SKDR", "Single simple beam with various boundary conditions", "TMarsupilami", "Core3Lib")
        {
            frames_0 = new List<MFrame>();
            frames_i = new List<MFrame>();
            N = 1;
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{D33CDD3A-6F3D-45B9-843C-CE520AB2D403}"); }
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
            pManager.RegisterParam(new Param_MBeam(), "Beams", "B", "Beam elements.");
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

        private void DR_Relax(int iteration_max)
        {
            var Ec_x_lim = 1e-10;
            var Ec_θ_lim = 1e-6;

            // INIT
            Rhino.RhinoApp.WriteLine("PB SETUP");
            Beam_4DOF_D beam = new Beam_4DOF_D(frames_0, frames_i, sections, materials);
            elements = new Beam[1] { beam };

            double M1 = 1 * 50 * 1e4;
            double M2 = 1 * 20 * -1e4;
            double Q = 0 * 1e6;
            double Fz = 1 * -1e6;
            double Fn = 0 * 1e3;

            int nmid = elements[0].Nv / 2;
            int nend = elements[0].Nv - 1;

            bool old = false;

            var bc_list = new List<Support>();
            switch (bc_start)
            {
                case (int)SupportCondition.Free:
                    //elements[0].Mext_m[0] = new MVector(M1, M2, 0);
                    //elements[0].Fext_g[0] = new MVector(0, 0, 0 * 1e6);
                    break;

                case (int)SupportCondition.Clamped:
                    if (old)
                    {
                        bc_list.Add(Support.AddClampedSupport(elements[0], Boundary.Start));
                    }
                    else
                    {
                        Support.AddClampedSupport(elements[0], Boundary.Start);
                    }
                    break;

                default:
                    if (old)
                    {
                        bc_list.Add(Support.AddPinnedSupport(elements[0], Boundary.Start));
                    }
                    else
                    {
                        Support.AddPinnedSupport(elements[0], Boundary.Start);
                    }
                    break;
            }

            var loads = new List<BeamVectorLoad>();

            switch (bc_end)
            {
                

                case (int)SupportCondition.Free:

                    // force verticale
                    loads.Add(BeamVectorLoad.Create_Fext(new MVector(0, 0, -1e5), Boundary.End, beam, true));
                    //loads.Add(BeamVectorLoad.Create_Fext(new MVector(0, 0, -1e5), beam.Nvg / 2, beam, true));
                    loads.Add(BeamVectorLoad.Create_Mext(new MVector(0, -1e5,0), beam.Nvg/2, beam, false));
                    loads.Add(BeamVectorLoad.Create_Mext(new MVector(0, 0,1e4), beam.Nvg/2, beam, false));


                    for (int i = 0; i < beam.Nvg; i++)
                    {
                        loads.Add(BeamVectorLoad.Create_mext(new MVector(0, 0, 5e3), i, beam, false));
                    }

                    //loads.Add(BeamVectorLoad.Create_fext(new MVector(0, 0, -1e4), 3, beam, true));
                    //loads.Add(BeamVectorLoad.Create_fext(new MVector(0, 0, 1e4), 4, beam, true));
                    //loads.Add(BeamVectorLoad.Create_mext(new MVector(0, -0.5 * 1e4, 0), beam.Nvh / 2 - 1, beam, false));

                    // force suiveuse
                    //loads.Add(BeamVectorLoad.Create_Fext(new MVector(-1e4, 0, 0), Boundary.End, beam, false));



                    //loads.Add(BeamVectorLoad.Create_Mext(Mext, Boundary.End, beam, true));

                    //loads.Add(BeamVectorLoad.Create_Fext(Fext, Boundary.End, beam, false));
                    //loads.Add(BeamVectorLoad.Create_fext(-Fext, Boundary.End, beam, false));
                    //loads.Add(BeamVectorLoad.Create_Mext(new MVector(0, 0, 1e5), Boundary.End, beam, false));
                    //loads.Add(BeamVectorLoad.Create_mext(0*new MVector(0,0,1e5), Boundary.End, beam, false));


                    //elements[0].Mext[elements[0].Nn - 1] = new MVector(M1, M2,Q);
                    //elements[0].Fext[elements[0].Nn - 1] = new MVector(0, 0, 0*1e4);
                    break;
                case (int)SupportCondition.Clamped:
                    if (old)
                    {
                        bc_list.Add(Support.AddClampedSupport(elements[0], Boundary.End));
                    }
                    else
                    {
                        Support.AddClampedSupport(elements[0], Boundary.End);
                    }
                    //loads.Add(BeamVectorLoad.Create_Mext(new MVector(0,0,1e6), beam.Nvh/2, beam, false));
                    //loads.Add(BeamVectorLoad.Create_Fext(new MVector(0, 1e6, 0), beam.Nvh / 2, beam, true));
                    break;
                default:
                    if (old)
                    {
                        bc_list.Add(Support.AddPinnedSupport(elements[0], Boundary.End));
                    }
                    else
                    {
                        Support.AddPinnedSupport(elements[0], Boundary.End);

                    }
                    break;
            }

            beam.Load(loads);

            //solver = new KDRSolver2(elements, bc_list, iteration_max, Ec_x_lim, Ec_θ_lim);
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
        }

    }



}
