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
    public class Comp_DisplayBeam_M : GH_Component
    {

        public Comp_DisplayBeam_M()
          : base("Bending Moment (M)", "M",
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
                return Resources.DisplayBeam_M;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{08D301B5-9E54-4645-9F9C-345F1AA77726}"); }
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
            pManager.AddParameter(new Param_MCMoment(), "Ml", "Ml", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Mr", "Mr", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (M1)", "D1", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (M2)", "D2", "", GH_ParamAccess.list);
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

            CMoment[] Ml, Mr, Mmid;
            Configuration config;
            MPoint[] startPoints, endPoints_1, endPoints_2;

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

            beam.Get_M(out Ml, out Mr);
            beam.Diagram_M(out startPoints, out endPoints_1, out endPoints_2, scale, config);
            var D1 = Diagram.GetOutlines(startPoints, endPoints_1);
            var D2 = Diagram.GetOutlines(startPoints, endPoints_2);

            DA.SetDataList(0, Ml);
            DA.SetDataList(1, Mr);
            DA.SetDataList(2, D1);
            DA.SetDataList(3, D2);
        }

    }
}
