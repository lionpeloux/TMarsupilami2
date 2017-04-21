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
    public class Comp_DisplayBeam_ε : GH_Component
    {

        public Comp_DisplayBeam_ε()
          : base("Beam ε", "ε",
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
                return Resources.DisplayBeam_ε;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{B05BDE82-B7AB-4277-970B-7D0D29893906}"); }
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
            pManager.AddNumberParameter("εl", "εl", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("εr", "εr", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("εmid", "εmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (ε)", "D", "", GH_ParamAccess.list);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();
            var scale = 1.0;
            var configIndex = 2;

            if (!DA.GetData(0, ref ghBeam)){ return; }

            DA.GetData(1, ref scale);
            DA.GetData(2, ref configIndex);

            var beam = ghBeam.Value as Beam_4DOF_D;

            double[] εl, εr, εmid;
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

            beam.Get_ε(out εl, out εr, out εmid);
            beam.Diagram_ε(out startPoints, out endPoints, scale, config, Axis.d1);
            var D = Diagram.GetOutlines(startPoints, endPoints);

            DA.SetDataList(0, εl);
            DA.SetDataList(1, εr);
            DA.SetDataList(2, εmid);
            DA.SetDataList(3, D);
        }

    }
}
