﻿using System;
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
    public class Comp_DisplayBeam_ε : GH_Component
    {

        public Comp_DisplayBeam_ε()
          : base("Beam ε", "ε",
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
            get { return new Guid("{B05BDE82-B7AB-4277-970B-7D0D29893906}"); }
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
            pManager.AddNumberParameter("εr", "εr", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("εl", "εl", "", GH_ParamAccess.list);
            pManager.AddNumberParameter("εmid", "εmid", "", GH_ParamAccess.list);
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

            double[] εl, εr, εmid;

            beam.Get2_ε(out εl, out εr, out εmid);

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

                var pt_l = (frame.Origin + scale * εl[i] * d).Cast();
                pts.Add(pt_l);
                diagram.Add((new Line(frame.Origin.Cast(), pt_l)).ToNurbsCurve());

                var pt_r = (frame.Origin + scale * εr[i] * d).Cast();
                pts.Add(pt_r);
                diagram.Add((new Line(frame.Origin.Cast(), pt_r)).ToNurbsCurve());
            }

            diagram.Add(new Polyline(pts).ToNurbsCurve());

            DA.SetDataList(0, εr);
            DA.SetDataList(1, εl);
            DA.SetDataList(2, εmid);
            DA.SetDataList(3, diagram);

        }

    }
}
