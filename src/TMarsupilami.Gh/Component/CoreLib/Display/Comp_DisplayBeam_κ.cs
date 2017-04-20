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
            pManager.AddBooleanParameter("Rest/Actual", "B", "Choose config", GH_ParamAccess.item, false);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MVector(), "κbl", "κbl", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "κbr", "κbr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MVector(), "κbmid", "κbmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("D1", "D2", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("D2", "D1", "", GH_ParamAccess.list);
        }

        protected override void BeforeSolveInstance()
        {
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();
            var scale = 1.0;
            var isRest = false;

            if (!DA.GetData(0, ref ghBeam)){ return; }

            DA.GetData(1, ref scale);
            DA.GetData(2, ref isRest);

            var beam = ghBeam.Value as Beam_4DOF_D;

            MVector[] κbl, κbr, κbmid;

            beam.Get2_κ(out κbl, out κbr, out κbmid);

            var pts_1 = new List<Point3d>();
            var diagram_1 = new List<NurbsCurve>();
            var pts_2 = new List<Point3d>();
            var diagram_2 = new List<NurbsCurve>();

            for (int i = 0; i < beam.Nv; i++)
            {
                MFrame frame;
                if (isRest)
                {
                    frame = beam.RestConfiguration[i];
                }
                else
                {
                    frame = beam.ActualConfiguration[i];
                }

                var d1 = frame.XAxis;
                var d2 = frame.YAxis;

                var κbl1 = scale * κbl[i] * beam.ActualConfiguration[i].XAxis;
                var pt_l = (frame.Origin + κbl1 * d1).Cast();
                pts_1.Add(pt_l);
                var line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram_1.Add(line_l.ToNurbsCurve());

                var κbr1 = scale * κbr[i] * beam.ActualConfiguration[i].XAxis;
                var pt_r = (frame.Origin + κbr1 * d1).Cast();
                pts_1.Add(pt_r);
                var line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram_1.Add(line_r.ToNurbsCurve());

                var κbl2 = scale * κbl[i] * beam.ActualConfiguration[i].YAxis;
                pt_l = (frame.Origin + κbl2 * d2).Cast();
                pts_2.Add(pt_l);
                line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram_2.Add(line_l.ToNurbsCurve());

                var κbr2 = scale * κbr[i] * beam.ActualConfiguration[i].YAxis;
                pt_r = (frame.Origin + κbr2 * d2).Cast();
                pts_2.Add(pt_r);
                line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram_2.Add(line_r.ToNurbsCurve());
            }

            diagram_1.Add(new Polyline(pts_1).ToNurbsCurve());
            diagram_2.Add(new Polyline(pts_2).ToNurbsCurve());

            DA.SetDataList(0, κbr);
            DA.SetDataList(1, κbl);
            DA.SetDataList(2, κbmid);
            DA.SetDataList(3, diagram_1);
            DA.SetDataList(4, diagram_2);

        }

    }
}
