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
    public class Comp_DisplayBeamN : GH_Component
    {

        public Comp_DisplayBeamN()
          : base("Beam N", "N",
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

            var Nr = new CForce[beam.Nv];
            for (int i = 0; i < beam.Nv; i++)
            {
                var N = beam.N_r[i] * beam.ActualConfiguration[i].ZAxis;
                Nr[i] = new CForce(N, beam.ActualConfiguration[i]);
            }

            var Nl = new CForce[beam.Nv];
            for (int i = 0; i < beam.Nv; i++)
            {
                var N = beam.N_l[i] * beam.ActualConfiguration[i].ZAxis;
                Nl[i] = new CForce(N, beam.ActualConfiguration[i]);
            }

            var Nmid = new CForce[beam.Ne];
            for (int i = 0; i < beam.Ne; i++)
            {
                var N = beam.N_mid[i] * beam.t_mid[i];
                Nmid[i] = new CForce(N, beam.mframes_mid[i]);
            }

            var pts = new List<Point3d>();
            var diagram = new List<NurbsCurve>();
            for (int i = 0; i < Nr.Length; i++)
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

                var pt_l = (frame.Origin + scale * Nl[i].Value.Length() * d).Cast();
                pts.Add(pt_l);
                diagram.Add((new Line(frame.Origin.Cast(), pt_l)).ToNurbsCurve());

                var pt_r = (frame.Origin + scale * Nr[i].Value.Length() * d).Cast();
                pts.Add(pt_r);
                diagram.Add((new Line(frame.Origin.Cast(), pt_r)).ToNurbsCurve());
            }

            diagram.Add(new Polyline(pts).ToNurbsCurve());

            DA.SetDataList(0, Nr);
            DA.SetDataList(1, Nl);
            DA.SetDataList(2, Nmid);
            DA.SetDataList(3, diagram);

        }

    }
}
