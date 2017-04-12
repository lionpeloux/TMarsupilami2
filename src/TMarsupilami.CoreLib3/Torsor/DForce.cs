using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    /// <summary>
    /// Represents a uniform distributed force (f) over a segment.
    /// </summary>
    /// <remarks>
    /// The segment is defined by its start and end points.
    /// A local frame is attached to the force to provide auto projection of its components in the given local frame.
    /// </remarks>
    public struct DForce : IDeepCopy<DForce>, IShallowCopy<DForce>
    {
        public MFrame LocalFrame { get; private set; }
        public MPoint StartPoint { get; private set; }
        public MPoint EndPoint { get; private set; }
        public MVector Value { get; private set; }

        public DForce(MVector valueInGCS, MPoint startPoint, MPoint endPoint, MFrame localFrameInGCS)
        {
            LocalFrame = localFrameInGCS;
            Value = valueInGCS;
            StartPoint = startPoint;
            EndPoint = endPoint;
        }
        public DForce(MVector valueInGCS, MPoint startPoint, MPoint endPoint) :this(valueInGCS, startPoint, endPoint, MFrame.XY)
        {
        }

        public void GetComponents(out MVector f1, out MVector f2, out MVector f3, bool inGCS)
        {
            if (inGCS)
            {
                f1 = new MVector(Value.X, 0, 0);
                f2 = new MVector(0, Value.Y, 0);
                f3 = new MVector(0, 0, Value.Z);
            }
            else
            {
                GetComponents(LocalFrame, out f1, out f2, out f3);
            }

        }
        public void GetComponents(MFrame localFrameInGCS, out MVector f1, out MVector f2, out MVector f3)
        {
            var valueInLCS = BasisChange.ToLocal(Value, localFrameInGCS);
            f1 = valueInLCS.X * localFrameInGCS.XAxis;
            f2 = valueInLCS.Y * localFrameInGCS.YAxis;
            f3 = valueInLCS.Z * localFrameInGCS.ZAxis;
        }

        public void GetCoordinates(out double f1, out double f2, out double f3, bool inGCS)
        {
            if (inGCS)
            {
                f1 = Value.X;
                f2 = Value.Y;
                f3 = Value.Z;
            }
            else
            {
                GetCoordinates(LocalFrame, out f1, out f2, out f3);
            }
        }
        public void GetCoordinates(MFrame localFrameInGCS, out double f1, out double f2, out double f3)
        {
            var valueInLCS = BasisChange.ToLocal(Value, localFrameInGCS);
            f1 = valueInLCS.X;
            f2 = valueInLCS.Y;
            f3 = valueInLCS.Z;
        }

        public override string ToString()
        {
            return "[T] = { O : " + LocalFrame.Origin + " | F : " + Value + " }";
        }
        public DForce DeepCopy()
        {
            return new DForce(Value, StartPoint, EndPoint, LocalFrame);
        }
        public DForce ShallowCopy()
        {
            return DeepCopy();
        }
    }

}
