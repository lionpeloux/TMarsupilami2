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
    public class Comp_DisplayBeam_Q : GH_Component
    {

        public Comp_DisplayBeam_Q()
          : base("Beam Q", "Q",
              "Extract Beam Internals Informations",
              "TMarsupilami", "Display")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.tertiary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.DisplayBeam_Q;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{02CCFBE4-0F42-4C47-A60B-C476C6C85659}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MBeam(), "Beam", "B", "The beam to preview.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scale", "S", "Scale", GH_ParamAccess.item, 1);
            pManager.AddIntegerParameter("Configuration", "C", "Rest (0), Initial (1), Actual (2).", GH_ParamAccess.item, 2);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MCMoment(), "Ql", "Ql", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Qr", "Qr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Qmid", "Qmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (Q)", "D", "", GH_ParamAccess.list);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();
            var scale = 1.0;
            var configIndex = 2;

            if (!DA.GetData(0, ref ghBeam)) { return; }

            DA.GetData(1, ref scale);
            DA.GetData(2, ref configIndex);

            var beam = ghBeam.Value as Beam_4DOF_D;

            CMoment[] Ql, Qr, Qmid;
            Configuration config;
            MPoint[] startPoints, endPoints;

            switch (configIndex)
            {
                case 0:
                    config = Configuration.Rest;
                    break;
                case 1:
                    config = Configuration.Initial;
                    break;
                default:
                    config = Configuration.Actual;
                    break;
            }

            beam.Get_Q(out Ql, out Qr, out Qmid);
            beam.Diagram_Q(out startPoints, out endPoints, scale, config, Axis.d1);
            var D = Diagram.GetOutlines(startPoints, endPoints);

            DA.SetDataList(0, Ql);
            DA.SetDataList(1, Qr);
            DA.SetDataList(2, Qmid);
            DA.SetDataList(3, D);
        }

    }
}
