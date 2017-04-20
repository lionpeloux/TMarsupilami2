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
    public class Comp_DisplayBeam_V : GH_Component
    {

        public Comp_DisplayBeam_V()
          : base("Beam V", "V",
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
            get { return new Guid("{253B90E5-C89E-43B0-9E80-3ECFFB465566}"); }
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
            pManager.AddParameter(new Param_MCForce(), "Vr", "Vr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Vl", "Vl", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Vmid", "Vmid", "", GH_ParamAccess.list);
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

            CForce[] Vr, Vl, Vmid;

            beam.Get2_V(out Vl, out Vr, out Vmid);

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

                var Vl1 = scale * Vl[i].Value * beam.ActualConfiguration[i].XAxis;
                var pt_l = (frame.Origin + Vl1 * d1).Cast();
                pts_1.Add(pt_l);
                var line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram_1.Add(line_l.ToNurbsCurve());

                var Vr1 = scale * Vr[i].Value * beam.ActualConfiguration[i].XAxis;
                var pt_r = (frame.Origin + Vr1 * d1).Cast();
                pts_1.Add(pt_r);
                var line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram_1.Add(line_r.ToNurbsCurve());

                var Vl2 = scale * Vl[i].Value * beam.ActualConfiguration[i].YAxis;
                pt_l = (frame.Origin + Vl2 * d2).Cast();
                pts_2.Add(pt_l);
                line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram_2.Add(line_l.ToNurbsCurve());

                var Vr2 = scale * Vr[i].Value * beam.ActualConfiguration[i].YAxis;
                pt_r = (frame.Origin + Vr2 * d2).Cast();
                pts_2.Add(pt_r);
                line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram_2.Add(line_r.ToNurbsCurve());
            }

            diagram_1.Add(new Polyline(pts_1).ToNurbsCurve());
            diagram_2.Add(new Polyline(pts_2).ToNurbsCurve());

            DA.SetDataList(0, Vr);
            DA.SetDataList(1, Vl);
            DA.SetDataList(2, Vmid);
            DA.SetDataList(3, diagram_1);
            DA.SetDataList(4, diagram_2);
        }

    }
}
