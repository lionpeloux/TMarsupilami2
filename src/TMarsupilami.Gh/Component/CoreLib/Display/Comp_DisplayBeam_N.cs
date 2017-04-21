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
    public class Comp_DisplayBeam_N : GH_Component
    {

        public Comp_DisplayBeam_N()
          : base("Beam N", "N",
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
            get { return new Guid("{A0F0CA02-2F62-4C96-A550-60424C645CB9}"); }
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
            pManager.AddParameter(new Param_MCForce(), "Nl", "Nl", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Nr", "Nr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Nmid", "Nmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("Diagram (N)", "D", "", GH_ParamAccess.list);
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

            CForce[] Nl, Nr, Nmid;
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

            beam.Get_N(out Nl, out Nr, out Nmid);
            beam.Diagram_N(out startPoints, out endPoints, scale, config, Axis.d1);

            int n = startPoints.Length;
            var pts = new Point3d[n];
            var diagram = new NurbsCurve[n + 1];

            for (int i = 0; i < startPoints.Length; i++)
            {
                var ps = startPoints[i].Cast();
                var pe = endPoints[i].Cast();
                var line = new Line(ps, pe);
                diagram[i] = line.ToNurbsCurve();
                pts[i] = pe;
            }

            diagram[n] = (new Polyline(pts)).ToNurbsCurve();

            DA.SetDataList(0, Nl);
            DA.SetDataList(1, Nr);
            DA.SetDataList(2, Nmid);
            DA.SetDataList(3, diagram);
        }

    }
}
