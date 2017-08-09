using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.CoreLib3;
using System.Diagnostics;
using Rhino;
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
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        // PARAMETERS
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("File Path", "path", "The path to the json file", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.RegisterParam(new Param_MBeam(), "Beams", "B", "Beam element", GH_ParamAccess.item);
        }

        // SOLVER
        protected override void SolveInstance(IGH_DataAccess DA)
        {
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
            switch (pb.End)
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
        }
    }



}
