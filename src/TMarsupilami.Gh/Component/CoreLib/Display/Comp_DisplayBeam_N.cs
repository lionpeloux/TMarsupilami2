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
            pManager.AddBooleanParameter("Rest/Actual", "B", "Choose config", GH_ParamAccess.item, false);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MCForce(), "Nr", "Nr", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Nl", "Nl", "", GH_ParamAccess.list);
            pManager.AddParameter(new Param_MCForce(), "Nmid", "Nmid", "", GH_ParamAccess.list);
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

            CForce[] Nr, Nl, Nmid;

            beam.Get2_N(out Nl, out Nr, out Nmid);

            var pts = new List<Point3d>();
            var diagram = new List<NurbsCurve>();
            for (int i = 0; i < Nr.Length; i++)
            {
                MFrame frame;
                MVector d;

                if (isRest)
                {
                    frame = beam.RestConfiguration[i];
                }
                else
                {
                    frame = beam.ActualConfiguration[i];
                }

                d = frame.XAxis;
                d.Normalize();

                var d3 = beam.ActualConfiguration[i].ZAxis;

                var Nl3 = scale * (Nl[i].Value * d3);
                var pt_l = (frame.Origin + Nl3 * d).Cast();
                pts.Add(pt_l);
                var line_l = new Line(frame.Origin.Cast(), pt_l);
                diagram.Add(line_l.ToNurbsCurve());
     
                var Nr3 = scale * (Nr[i].Value * d3);
                var pt_r = (frame.Origin + Nr3 * d).Cast();
                pts.Add(pt_r);
                var line_r = new Line(frame.Origin.Cast(), pt_r);
                diagram.Add(line_r.ToNurbsCurve());
            }

            diagram.Add(new Polyline(pts).ToNurbsCurve());

            DA.SetDataList(0, Nr);
            DA.SetDataList(1, Nl);
            DA.SetDataList(2, Nmid);
            DA.SetDataList(3, diagram);

        }

    }
}
