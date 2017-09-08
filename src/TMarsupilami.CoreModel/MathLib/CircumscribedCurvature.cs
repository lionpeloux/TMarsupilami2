using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public class CircumscribedCurvature : Curvature
    {
        public override void Evaluate(MPoint ps, MPoint p, MPoint pe, out double κ, out MVector κb, out MVector ts, out MVector t, out MVector te, out double fs, out double f, out double fe)
        {
            throw new NotImplementedException();
        }

        public override void Evaluate(MVector startTangent, MPoint startPoint, MPoint currentPoint)
        {
            throw new NotImplementedException();
        }

        public override void Evaluate(MPoint currentPoint, MPoint endPoint, MVector endTangent)
        {
            throw new NotImplementedException();
        }
    }
}