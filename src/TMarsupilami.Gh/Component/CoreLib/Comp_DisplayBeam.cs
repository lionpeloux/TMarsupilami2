using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;
using System.Drawing;
using GH_IO.Types;
using Grasshopper;
using TMarsupilami.Gh.Parameter;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Type;
using TMarsupilami.CoreLib3;

namespace TMarsupilami.Gh.Component
{
    public class Comp_DisplayBeam : GH_Component
    {

        public Comp_DisplayBeam()
          : base("Beam Internals", "BDis",
              "Extract Beam Internals Informations",
              "TMarsupilami", "Display")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.primary;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{80D808A8-9980-4392-AA89-FA8E052782B0}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MBeam(), "Beam", "B", "The beam to preview.", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The material frames of the beam.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "N", "N", "The axial force.", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MForce(), "N", "N", "The axial force.", GH_ParamAccess.list);

        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();

            if (!DA.GetData(0, ref ghBeam)){ return; }

            var beam = ghBeam.Value;

            var Fext = new Force[beam.Fext_g.Length];

            for (int i = 0; i < Fext.Length; i++)
            {
                Fext[i] = new Force(beam.Fext_g[i], beam.ActualConfiguration[2 * i], true);
            }

            DA.SetDataList(0, beam.ActualConfiguration);
            DA.SetDataList(1, beam.Fext_g);
            DA.SetDataList(2, Fext);

        }

    }
}
