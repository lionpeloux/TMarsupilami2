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
    public class Comp_WriteSingleBeamProblem : GH_Component
    {

        // CONSTRUCTOR
        public Comp_WriteSingleBeamProblem()
            : base("Write Single Beam Problem", "Write SBP", "Write a SingleBeamProblem into a json file", "TMarsupilami", "Bench")
        {
        } 
        public override Guid ComponentGuid
        {
            get { return new Guid("{60BD5605-B22B-4F0D-85F6-4CD73D8AE970}"); }
        }
        public override GH_Exposure Exposure
        {
            get { return GH_Exposure.primary; }
        }

        // PARAMETERS
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("File Path", "path", "The path to the json file", GH_ParamAccess.item);

            pManager.AddParameter(new Param_MFrame(), "Rest Configuration", "sections_0", "Beam's sections in rest position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MFrame(), "Actual Configuration", "sections_i", "Beam's sections in actual position. MFrame z_axis is supposed to be aligned with d3 material vector.", GH_ParamAccess.list);

            pManager.AddNumberParameter("Rectangular Section Width", "b1", "", GH_ParamAccess.item);
            pManager.AddNumberParameter("Rectangular Section Height", "b2", "", GH_ParamAccess.item);

            pManager.AddIntegerParameter("Boundary Condition at Start", "bc_start", "(FREE = 0, PINNED = 1, CLAMPED = 2)", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("Boundary Condition at End", "bc_end", "(FREE = 0, PINNED = 1, CLAMPED  =2)", GH_ParamAccess.item, 1);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("JSON", "json", "output string in JSON format.", GH_ParamAccess.item);
        }

        // SOLVER
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string path = "";
            var frames_0 = new List<MFrame>();
            var frames_i = new List<MFrame>();

            double b1 = 0.0;
            double b2 = 0.0;

            int bc_start = 0;
            int bc_end = 0;

            if (!DA.GetData(0, ref path)) { return; }

            if (!DA.GetDataList(1, frames_0)) { return; }
            if (!DA.GetDataList(2, frames_i)) { return; }

            if (!DA.GetData(3, ref b1)) { return; }
            if (!DA.GetData(4, ref b2)) { return; }

            if (!DA.GetData(5, ref bc_start)) { return; }
            if (!DA.GetData(6, ref bc_end)) { return; }

            var pb = new SingleBeamProblem(frames_0, frames_i, bc_start, bc_end, b1, b2, 0, 0);
            var json = SingleBeamProblem.Serialize(pb, path);

            DA.SetData(0, json);        
        }
    }



}
