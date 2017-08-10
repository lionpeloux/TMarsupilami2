using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public abstract class Curvature
    {

        // CURRENT
        public abstract void Evaluate(MPoint ps, MPoint p, MPoint pe, out double κ, out MVector κb, out MVector ts, out MVector t, out MVector te, out double fs, out double f, out double fe);

        // START
        public abstract void Evaluate(MVector startTangent, MPoint startPoint, MPoint currentPoint);

        // END
        public abstract void Evaluate(MPoint currentPoint, MPoint endPoint, MVector endTangent);
    }
}