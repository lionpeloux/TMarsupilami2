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
    public class Comp_DisplayBeam_κ : GH_Component
    {

        public Comp_DisplayBeam_κ()
          : base("Beam κ", "κ",
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
        public override Guid ComponentGuid
        {
            get { return new Guid("{96F95950-53F0-4BBA-8B56-0C466EE0A1A0}"); }
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
            pManager.AddParameter(new Param_MVector(), "κl", "κl", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "κr", "κr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "κmid", "κmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (κ1)", "D1", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (κ2)", "D2", "", GH_ParamAccess.list);
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

            MVector[] κl, κr, κmid;
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

            beam.Get_κ(out κl, out κr, out κmid);
            beam.Diagram_κ(out startPoints, out endPoints_1, out endPoints_2, scale, config);

            int n = startPoints.Length;
            var pts_1 = new Point3d[n];
            var pts_2 = new Point3d[n];
            var diagram_1 = new NurbsCurve[n + 1];
            var diagram_2 = new NurbsCurve[n + 1];

            for (int i = 0; i < startPoints.Length; i++)
            {
                var ps = startPoints[i].Cast();
                var pe1 = endPoints_1[i].Cast();
                var pe2 = endPoints_2[i].Cast();

                var l1 = new Line(ps, pe1);
                var l2 = new Line(ps, pe2);

                diagram_1[i] = l1.ToNurbsCurve();
                diagram_2[i] = l2.ToNurbsCurve();

                pts_1[i] = pe1;
                pts_2[i] = pe2;
            }

            diagram_1[n] = new Polyline(pts_1).ToNurbsCurve();
            diagram_2[n] = new Polyline(pts_2).ToNurbsCurve();


            DA.SetDataList(0, κl);
            DA.SetDataList(1, κr);
            DA.SetDataList(2, κmid);
            DA.SetDataList(3, diagram_1);
            DA.SetDataList(4, diagram_2);
        }
    }
}
