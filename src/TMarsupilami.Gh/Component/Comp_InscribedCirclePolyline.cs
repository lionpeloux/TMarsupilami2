using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using TMarsupilami.Gh.Properties;

namespace TMarsupilami.Gh
{
    public class Comp_InscribedCirclePolyline : GH_Component
    {

        public Comp_InscribedCirclePolyline()
          : base("Inscribed Circle", "Inscribed Circle",
              "Circle tangent to two edges. Applies to a polyline.",
              "TMarsupilami", "Math")
        {
        }

        public override GH_Exposure Exposure
        {
            get
            {
                return GH_Exposure.secondary;
            }
        }
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Resources.InscribedCirclePolyline;
            }
        }
        public override Guid ComponentGuid
        {
            get { return new Guid("{A6BDD2DF-63D2-4C01-9C59-5423FD0BD769}"); }
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Polyline", "P", "Polyline.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Tangent at Start", "ts", "Tangent at Start.", GH_ParamAccess.item);
            pManager.AddVectorParameter("Tangent at End", "te", "Tangent at End.", GH_ParamAccess.item);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Curvature", "k", "Circle curvature", GH_ParamAccess.list);
            pManager.AddVectorParameter("Curvature Binormal Vector", "kb", "Circle curvature bionormal vector.", GH_ParamAccess.list);
            pManager.AddVectorParameter("Unit Tangent Vector at P", "t", "Circle unit tangent vector at P.", GH_ParamAccess.list);
            pManager.AddGeometryParameter("Circle", "C", "Circle tangent to two consecutive edges. Can be a line if points are aligned.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve curve = null;
            Polyline polyline;
            var t_start = new Vector3d();
            var t_end = new Vector3d();

            if (!DA.GetData(0, ref curve)) { return; }
            if (!DA.GetData(1, ref t_start)) { t_start = new Vector3d(0, 0, 0); }
            if (!DA.GetData(2, ref t_end)) { t_end = new Vector3d(0, 0, 0); }

            if (!curve.TryGetPolyline(out polyline))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input curve is not a Polyline.");
                return;
            }
            double[] κ_list;
            Vector3d[] κb_list;
            Vector3d[] t_list;
            Curve[] g_list;

            for (int i = 0; i < polyline.Count - 1; i++)
            {
                if (polyline[i] == polyline[i + 1])
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Polyline vertices must be disjoints.");
                    return;
                }
            }

            if (polyline.IsClosed)
            {
                GetInscribedCircle_Close(polyline, t_start, t_end, out κ_list, out κb_list, out t_list, out g_list);
            }
            else
            {
                GetInscribedCircle_Open(polyline, t_start, t_end, out κ_list, out κb_list, out t_list, out g_list);
            }

            DA.SetDataList(0, κ_list);
            DA.SetDataList(1, κb_list);
            DA.SetDataList(2, t_list);
            DA.SetDataList(3, g_list);
        }

        private void GetInscribedCircle_Open(Polyline polyline, Vector3d t_start, Vector3d t_end, out double[] κ_list, out Vector3d[] κb_list, out Vector3d[] t_list, out Curve[] g_list)
        {
            int N = polyline.Count;

            κ_list = new double[N];
            κb_list = new Vector3d[N];
            t_list = new Vector3d[N];
            g_list = new Curve[N];

            double κ;
            MathLib.Vector κb;
            MathLib.Vector ts, t, te;
            double fs, f, fe;

            // START
            if (t_start == new Vector3d(0, 0, 0))
            {
                κ_list[0] = 0;
                κb_list[0] = new Vector3d(0, 0, 0);
                t_list[0] = new Vector3d(0, 0, 0);
                g_list[0] = new ArcCurve(new Circle(polyline[0], 0));
            }
            else
            {
                t_start.Unitize();
                MathLib.Circle.InscribedCircle_Start(t_start.Cast(), polyline[0].Cast(), polyline[1].Cast(), out κ, out κb, out fs);
                double r = 1 / κ;
                var b = r * κb;
                var n = MathLib.Vector.CrossProduct(b, t_start.Cast());
                var center = polyline[0].Cast() + (r / Math.Cos(fs)) * n;
                var plane = new Plane(center.Cast(), t_start, n.Cast());
                var circle = new Circle(plane, center.Cast(), r);

                κ_list[0] = κ;
                κb_list[0] = κb.Cast();
                t_list[0] = t_start;
                g_list[0] = new ArcCurve(circle);
            }

            // CURRENT
            for (int i = 1; i < polyline.Count - 1; i++)
            {
                MathLib.Circle.InscribedCircle_Current(polyline[i - 1].Cast(), polyline[i].Cast(), polyline[i + 1].Cast(), out κ, out κb, out t, out f);

                if (κ == 0) // it's a line
                {
                    var line = new Line(polyline[i], t.Cast());
                    g_list[i] = new LineCurve(line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MathLib.Vector.CrossProduct(b, t);
                    var center = polyline[i].Cast() + (r / Math.Cos(f / 2)) * n;
                    var plane = new Plane(center.Cast(), t.Cast(), n.Cast());
                    var circle = new Circle(plane, center.Cast(), r);
                    g_list[i] = new ArcCurve(circle);
                }

                κ_list[i] = κ;
                κb_list[i] = κb.Cast();
                t_list[i] = t.Cast();

            }

            // END
            if (t_end == new Vector3d(0, 0, 0))
            {
                κ_list[N - 1] = 0;
                κb_list[N - 1] = new Vector3d(0, 0, 0);
                t_list[N - 1] = new Vector3d(0, 0, 0);
                g_list[N - 1] = new ArcCurve(new Circle(polyline[N - 1], 0));
            }
            else
            {
                t_end.Unitize();
                MathLib.Circle.InscribedCircle_End(polyline[N - 2].Cast(), polyline[N - 1].Cast(), t_end.Cast(), out κ, out κb, out fe);
                double r = 1 / κ;
                var b = r * κb;
                var n = MathLib.Vector.CrossProduct(b, t_end.Cast());
                var center = polyline[N - 1].Cast() + (r / Math.Cos(fe)) * n;
                var plane = new Plane(center.Cast(), t_end, n.Cast());
                var circle = new Circle(plane, center.Cast(), r);

                κ_list[N - 1] = κ;
                κb_list[N - 1] = κb.Cast();
                t_list[N - 1] = t_end;
                g_list[N - 1] = new ArcCurve(circle);
            }
        }
        private void GetInscribedCircle_Close(Polyline polyline, Vector3d t_start, Vector3d t_end, out double[] κ_list, out Vector3d[] κb_list, out Vector3d[] t_list, out Curve[] g_list)
        {
            int N = polyline.Count;

            κ_list = new double[N];
            κb_list = new Vector3d[N];
            t_list = new Vector3d[N];
            g_list = new Curve[N];

            double κ;
            MathLib.Vector κb;
            MathLib.Vector t;
            double f;

            var p_list = new List<Point3d>();
            p_list.Add(polyline[polyline.Count - 2]);
            p_list.AddRange(polyline);

            // CURRENT
            for (int i = 1; i < polyline.Count; i++)
            {
                MathLib.Circle.InscribedCircle_Current(p_list[i - 1].Cast(), p_list[i].Cast(), p_list[i + 1].Cast(), out κ, out κb, out t, out f);

                if (κ == 0) // it's a line
                {
                    var line = new Line(p_list[i], t.Cast());
                    g_list[i-1] = new LineCurve(line);
                }
                else // it's a circle
                {
                    double r = 1 / κ;
                    var b = r * κb;
                    var n = MathLib.Vector.CrossProduct(b, t);
                    var center = p_list[i].Cast() + (r / Math.Cos(f / 2)) * n;
                    var plane = new Plane(center.Cast(), t.Cast(), n.Cast());
                    var circle = new Circle(plane, center.Cast(), r);
                    g_list[i-1] = new ArcCurve(circle);
                }

                κ_list[i-1] = κ;
                κb_list[i-1] = κb.Cast();
                t_list[i-1] = t.Cast();
            }
        }


    }
}
