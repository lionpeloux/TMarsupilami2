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
    public class Comp_DisplayBeam_Geometry : GH_Component
    {

        public Comp_DisplayBeam_Geometry()
              : base("Material Frames", "Geometry",
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
        //protected override System.Drawing.Bitmap Icon
        //{
        //    get
        //    {
        //        return Resources.DisplayBeam_E;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{D4D9497E-1685-42E6-8E9D-6689B1AECB83}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MBeam(), "Beam", "B", "The beam to preview.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Configuration", "C", "Rest (0), Initial (1), Actual (2).", GH_ParamAccess.item, 2);

            pManager[1].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Material Frames", "F", "", GH_ParamAccess.list);
            pManager.AddBrepParameter("Envelope 3D", "brep", "Envelope 3D", GH_ParamAccess.list);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frames = new List<MFrame>();
            var ghBeam = new GH_MBeam();
            var configIndex = 2;

            if (!DA.GetData(0, ref ghBeam)) { return; }

            DA.GetData(1, ref configIndex);

            var beam = ghBeam.Value as Beam_4DOF_D;

            Configuration config;
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


            DA.SetDataList(0, beam.Get_MaterialFrames(config));
            DA.SetDataList(1, Envelope3D(beam, config));


        }


        private List<Brep> Envelope3D(Beam_4DOF_D beam, Configuration config)
        {
            var frames = beam.Get_MaterialFrames(config);
            var sections = beam.Sections;

            var points = new Point3d[beam.Nv];
            for (int i = 0; i < beam.Nv; i++)
            {
                points[i] = frames[i].Origin.Cast();
            }

            var centerlineCurve = Curve.CreateInterpolatedCurve(points, 3, CurveKnotStyle.ChordSquareRoot, frames[0].ZAxis.Cast(), frames[beam.Nv - 1].ZAxis.Cast());

            var param = new double[beam.Nvh-2];
            for (int i = 1; i < beam.Nvh-1; i++)
            {
                centerlineCurve.ClosestPoint(points[2 * i], out param[i - 1]);
            }

            var segmentCurves = centerlineCurve.Split(param);
            var envelope = new List<Brep>();

            var sweep = new Rhino.Geometry.SweepOneRail();
            sweep.ClosedSweep = false;
            sweep.SetToRoadlikeTop();

            for (int i = 0; i < beam.Nvh-1; i++)
            {
                var b1 = sections[i].b1;
                var b2 = sections[i].b2;

                var xsections = new Curve[2];
                xsections[0] = GetRectangularSection(frames[2*i], b1, b2);
                xsections[1] = GetRectangularSection(frames[2*i + 2], b1, b2);

                var breps = sweep.PerformSweep(segmentCurves[i], xsections);
                envelope.AddRange(breps);

            }

            return envelope;
        }

        private Curve GetRectangularSection(MFrame frame, double b1, double b2)
        {
            var p1 = (frame.Origin - b1 / 2 * frame.XAxis - b2 / 2 * frame.YAxis).Cast();
            var p2 = (frame.Origin + b1 / 2 * frame.XAxis - b2 / 2 * frame.YAxis).Cast();
            var p3 = (frame.Origin + b1 / 2 * frame.XAxis + b2 / 2 * frame.YAxis).Cast();
            var p4 = (frame.Origin - b1 / 2 * frame.XAxis + b2 / 2 * frame.YAxis).Cast();
            var polyline = new PolylineCurve(new Point3d[5] { p1, p2, p3, p4, p1});

            return polyline;

        }
    }
}
