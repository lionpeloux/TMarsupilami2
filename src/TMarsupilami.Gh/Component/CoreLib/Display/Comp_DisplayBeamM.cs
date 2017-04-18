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
    public class Comp_DisplayBeamM : GH_Component
    {

        public Comp_DisplayBeamM()
          : base("Beam M", "M",
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

            var Mext = new CMoment[beam.Mext_m.Length];
            for (int i = 0; i < Mext.Length; i++)
            {
                var M = beam.ToGlobalCoordinateSystem(beam.Mext_m[i], 2 * i);
                Mext[i] = new CMoment(M, beam.ActualConfiguration[2 * i]);
            }

            var Mr = new CMoment[beam.Nv];
            var Ml = new CMoment[beam.Nv];
            for (int i = 0; i < beam.Nvh; i++)
            {
                var M = beam.ToGlobalCoordinateSystem(beam.M_h_l[i], 2 * i);
                Ml[2 * i] = new CMoment(M, beam.ActualConfiguration[2 * i]);

                M = beam.ToGlobalCoordinateSystem(beam.M_h_r[i], 2 * i);
                Mr[2 * i] = new CMoment(M, beam.ActualConfiguration[2 * i]);
            }
            for (int i = 0; i < beam.Nvg; i++)
            {
                var M = beam.ToGlobalCoordinateSystem(beam.M_g[i], 2 * i + 1);
                Ml[2 * i + 1] = new CMoment(M, beam.ActualConfiguration[2 * i + 1]);
                Mr[2 * i + 1] = new CMoment(M, beam.ActualConfiguration[2 * i + 1]);
            }

         

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

                var pt_l = (frame.Origin + scale * Ml[i].Value.X * d1).Cast();
                pts_1.Add(pt_l);
                diagram_1.Add((new Line(frame.Origin.Cast(), pt_l)).ToNurbsCurve());

                var pt_r = (frame.Origin + scale * Mr[i].Value.X * d1).Cast();
                pts_1.Add(pt_r);
                diagram_1.Add((new Line(frame.Origin.Cast(), pt_r)).ToNurbsCurve());

                pt_l = (frame.Origin + scale * Ml[i].Value.Y * d2).Cast();
                pts_2.Add(pt_l);
                diagram_2.Add((new Line(frame.Origin.Cast(), pt_l)).ToNurbsCurve());

                pt_r = (frame.Origin + scale * Mr[i].Value.Y * d2).Cast();
                pts_2.Add(pt_r);
                diagram_2.Add((new Line(frame.Origin.Cast(), pt_r)).ToNurbsCurve());
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
