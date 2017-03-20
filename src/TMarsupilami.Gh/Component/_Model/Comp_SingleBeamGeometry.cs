using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.MathLib;
using TMarsupilami.Gh.Parameter;

namespace TMarsupilami.Gh.Component
{
    public class Comp_SingleBeamGeometry : GH_Component
    {

        public Comp_SingleBeamGeometry()
          : base("Define Beam Geometry - Single", "Geom",
              "Define the geometry of a single beam.",
              "TMarsupilami", "Single Beam")
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
        //        return Resources.ZRotatePlane_Diff;
        //    }
        //}
        public override Guid ComponentGuid
        {
            get { return new Guid("{8FC26282-FB27-4409-B7BC-B671EEB43A4A}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new Param_MFrame(), "Start Frame", "Fs", "The first frame of the beam.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MFrame(), "End Frame", "Fe", "The last frame of the beam.", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Edge Count", "N", "The total number of edges in the discretization.", GH_ParamAccess.item, 10);
            pManager.AddIntegerParameter("Twist Modulo", "M", "The number of additional full turns around the centerline.", GH_ParamAccess.item, 0);

            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Beam Centerline", "C", "The beam centerline as an interpolated curve.", GH_ParamAccess.item);
            pManager.AddParameter(new Param_MFrame(), "Frames", "F", "The discrete frames along the centerline.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var frameStart = new MFrame();
            var frameEnd = new MFrame();
            var N = 10;
            var twistModulo = 0;

            if (!DA.GetData(0, ref frameStart)) { return; }
            if (!DA.GetData(1, ref frameEnd)) { return; }
            if (!DA.GetData(2, ref N)) { }
            if (!DA.GetData(3, ref twistModulo)) { }

            var centerline = Curve.CreateInterpolatedCurve(
                            new Point3d[2] { frameStart.Origin.Cast(), frameEnd.Origin.Cast() },
                            3,
                            CurveKnotStyle.Uniform,
                            frameStart.ZAxis.Cast(),
                            frameEnd.ZAxis.Cast()
                        );

            // Subdivide centerline in N segments of equal length
            Point3d[] points;
            double[] verticesParameter;


            verticesParameter = centerline.DivideByCount(N, true, out points);
            var vectors = new Vector3d[N + 1];
            for (int i = 0; i < N + 1; i++)
            {
                vectors[i] = centerline.TangentAt(verticesParameter[i]);
            }

            // Compute edges length and total discretized length
            var l = new double[N];
            double L = 0;
            for (int i = 1; i < N + 1; i++)
            {
                l[i-1] = points[i - 1].DistanceTo(points[i]);
                L += l[i-1];
            }

            // Cast to MathLib types
            var vertices = points.Cast();
            var t = vectors.Cast();

            // Parallel tranpsort frameStart along the curve
            var framesPT = new MFrame[N + 1];
            framesPT[0] = frameStart;
            for (int i = 1; i < N + 1; i++)
            {
                framesPT[i - 1].ZParallelTransport_Rotation(t[i - 1], vertices[i], t[i], ref framesPT[i]);
            }

            // Compute closure angle in ]-pi;pi]
            double twistClosure = Rotation.ZAngle(frameEnd.XAxis, frameEnd.YAxis, framesPT[N].XAxis);
            double τ_closure = twistClosure / L;
            double τ_modulo = (2 * Math.PI) * twistModulo / L;
            double twist = 0;

            // Generate a set of frames that respects the closure angle
            for (int i = 1; i < N + 1; i++)
            {
                twist += (τ_closure + τ_modulo) * l[i - 1];
                framesPT[i].ZRotate(twist);
            }

            DA.SetData(0, centerline);
            DA.SetDataList(1, framesPT);
        }
    }
}
