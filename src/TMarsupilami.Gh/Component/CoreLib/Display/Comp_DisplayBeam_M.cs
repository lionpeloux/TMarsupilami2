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
          : base("Beam M", "M",
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
            get { return new Guid("{08D301B5-9E54-4645-9F9C-345F1AA77726}"); }
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
            pManager.AddParameter(new Param_MCMoment(), "Mr", "Mr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Ml", "Ml", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("D1", "D1", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("D2", "D2", "", GH_ParamAccess.list);
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
            CMoment[] Mr, Ml;

            beam.Get2_M(out Ml, out Mr);

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

                var Ml1 = scale * Ml[i].Value * beam.ActualConfiguration[i].XAxis;
                var pt_l = (frame.Origin + Ml1 * d1).Cast();
                pts_1.Add(pt_l);
                var line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram_1.Add(line_l.ToNurbsCurve());

                var Mr1 = scale * Mr[i].Value * beam.ActualConfiguration[i].XAxis;
                var pt_r = (frame.Origin + Mr1 * d1).Cast();
                pts_1.Add(pt_r);
                var line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram_1.Add(line_r.ToNurbsCurve());

                var Ml2 = scale * Ml[i].Value * beam.ActualConfiguration[i].YAxis;
                pt_l = (frame.Origin + Ml2 * d2).Cast();
                pts_2.Add(pt_l);
                line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram_2.Add(line_l.ToNurbsCurve());

                var Mr2 = scale * Mr[i].Value * beam.ActualConfiguration[i].YAxis;
                pt_r = (frame.Origin + Mr2 * d2).Cast();
                pts_2.Add(pt_r);
                line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram_2.Add(line_r.ToNurbsCurve());
            }

            diagram_1.Add(new Polyline(pts_1).ToNurbsCurve());
            diagram_2.Add(new Polyline(pts_2).ToNurbsCurve());

            DA.SetDataList(0, Mr);
            DA.SetDataList(1, Ml);
            DA.SetDataList(2, diagram_1);
            DA.SetDataList(3, diagram_2);
        }

    }
}
