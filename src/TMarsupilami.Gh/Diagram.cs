using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.Gh
{
    public static class Diagram
    {
        public static NurbsCurve[] GetOutlines(MPoint[] startPoints, MPoint[] endPoints)
        {
            int n = startPoints.Length;

            if (n != endPoints.Length)
                throw new ArgumentException("startPoints and endPoints arrays must have the same number of items");

            var pts = new Point3d[n];
            var outlines = new NurbsCurve[n + 1];

            for (int i = 0; i < n; i++)
            {
                var ps = startPoints[i].Cast();
                var pe = endPoints[i].Cast();
                var line = new Line(ps, pe);
                outlines[i] = line.ToNurbsCurve();
                pts[i] = pe;
            }

            outlines[n] = (new Polyline(pts)).ToNurbsCurve();
            return outlines;
        }
    }
}
