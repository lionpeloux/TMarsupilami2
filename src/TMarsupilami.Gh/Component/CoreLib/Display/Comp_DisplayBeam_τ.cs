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
    public class Comp_DisplayBeam_τ : GH_Component
    {

        public Comp_DisplayBeam_τ()
          : base("Beam τ", "τ",
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
            get { return new Guid("{4A37DEBC-93B1-4AB4-A00C-013F991D3B8B}"); }
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
            pManager.AddNumberParameter("τl", "τl", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("τr", "τr", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("τmid", "τmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (τ)", "D", "", GH_ParamAccess.list);
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

            double[] τl, τr, τmid;
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

            beam.Get_τ(out τl, out τr, out τmid);
            beam.Diagram_τ(out startPoints, out endPoints, scale, config, Axis.d1);

            var pts = new List<Point3d>();
            var diagram = new List<NurbsCurve>();

            for (int i = 0; i < startPoints.Length; i++)
            {
                var ps = startPoints[i].Cast();
                var pe = endPoints[i].Cast();
                var line = new Line(ps, pe);
                diagram.Add(line.ToNurbsCurve());
                pts.Add(pe);
            }

            diagram.Add(new Polyline(pts).ToNurbsCurve());

            DA.SetDataList(0, τl);
            DA.SetDataList(1, τr);
            DA.SetDataList(2, τmid);
            DA.SetDataList(3, diagram);
        }

    }
}
