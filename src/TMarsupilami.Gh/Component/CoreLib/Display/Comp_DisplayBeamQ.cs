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
    public class Comp_DisplayBeamQ : GH_Component
    {

        public Comp_DisplayBeamQ()
          : base("Beam Q", "Q",
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
            get { return new Guid("{02CCFBE4-0F42-4C47-A60B-C476C6C85659}"); }
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
            pManager.AddParameter(new Param_MCMoment(), "Qr", "Qr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Ql", "Ql", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCMoment(), "Qmid", "Qmid", "", GH_ParamAccess.list);
            pManager.AddCurveParameter("D", "D", "", GH_ParamAccess.list);
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
            CMoment[] Qr, Ql, Qmid;

            //Qr = new CMoment[beam.Nv];
            //Ql = new CMoment[beam.Nv];
            //Qmid = new CMoment[beam.Ne];

            //for (int i = 0; i < beam.Nv; i++)
            //{
            //    var Q = beam.Q_l[i] * beam.ActualConfiguration[i].ZAxis;
            //    Ql[i] = new CMoment(Q, beam.ActualConfiguration[i]);

            //    Q = beam.Q_r[i] * beam.ActualConfiguration[i].ZAxis;
            //    Qr[i] = new CMoment(Q, beam.ActualConfiguration[i]);
            //}

            //for (int i = 0; i < beam.Ne; i++)
            //{
            //    var Q = beam.Q_mid[i] * beam.t_mid[i];
            //    Qmid[i] = new CMoment(Q, beam.mframes_mid[i]);
            //}

            beam.Get_Q(out Ql, out Qr, out Qmid);

            var pts = new List<Point3d>();
            var diagram = new List<NurbsCurve>();
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

                var d = frame.XAxis;

                var pt_l = (frame.Origin + scale * Ql[i].Value.Length() * d).Cast();
                pts.Add(pt_l);
                diagram.Add((new Line(frame.Origin.Cast(), pt_l)).ToNurbsCurve());

                var pt_r = (frame.Origin + scale * Qr[i].Value.Length() * d).Cast();
                pts.Add(pt_r);
                diagram.Add((new Line(frame.Origin.Cast(), pt_r)).ToNurbsCurve());
            }

            diagram.Add(new Polyline(pts).ToNurbsCurve());

            DA.SetDataList(0, Qr);
            DA.SetDataList(1, Ql);
            DA.SetDataList(2, Qmid);
            DA.SetDataList(3, diagram);

        }

    }
}
