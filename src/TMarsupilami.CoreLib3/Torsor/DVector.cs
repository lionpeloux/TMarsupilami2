using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{

    public class DVector : IDeepCopy<DVector>
    {
        public MFrame LocalFrame { get; private set; }
        public MPoint StartPoint { get; private set; }
        public MPoint EndPoint { get; private set; }
        public MVector Value { get; private set; }

        public DVector(MVector valueInGCS, MPoint startPoint, MPoint endPoint, MFrame localFrameInGCS)
        {
            LocalFrame = localFrameInGCS;
            Value = valueInGCS;
            StartPoint = startPoint;
            EndPoint = endPoint;
        }
        public DVector(MVector valueInGCS, MPoint startPoint, MPoint endPoint) :this(valueInGCS, startPoint, endPoint, MFrame.XY)
        {
        }

        public void GetComponents(out MVector v1, out MVector v2, out MVector v3, bool inGCS)
        {
            if (inGCS)
            {
                v1 = new MVector(Value.X, 0, 0);
                v2 = new MVector(0, Value.Y, 0);
                v3 = new MVector(0, 0, Value.Z);
            }
            else
            {
                GetComponents(LocalFrame, out v1, out v2, out v3);
            }

        }
        public void GetComponents(MFrame localFrameInGCS, out MVector v1, out MVector v2, out MVector v3)
        {
            var valueInLCS = BasisChange.ToLocal(Value, localFrameInGCS);
            v1 = valueInLCS.X * localFrameInGCS.XAxis;
            v2 = valueInLCS.Y * localFrameInGCS.YAxis;
            v3 = valueInLCS.Z * localFrameInGCS.ZAxis;
        }

        public void GetCoordinates(out double v1, out double v2, out double v3, bool inGCS)
        {
            if (inGCS)
            {
                v1 = Value.X;
                v2 = Value.Y;
                v3 = Value.Z;
            }
            else
            {
                GetCoordinates(LocalFrame, out v1, out v2, out v3);
            }
        }
        public void GetCoordinates(MFrame localFrameInGCS, out double v1, out double v2, out double v3)
        {
            var valueInLCS = BasisChange.ToLocal(Value, localFrameInGCS);
            v1 = valueInLCS.X;
            v2 = valueInLCS.Y;
            v3 = valueInLCS.Z;
        }

        public override string ToString()
        {
            return "[T] = { O : " + LocalFrame.Origin + " | V : " + Value + " }";
        }
        public DVector DeepCopy()
        {
            return new DVector(Value, StartPoint, EndPoint, LocalFrame);
        }
    }

    /// <summary>
    /// Represents a uniform distributed force (f) over a segment.
    /// </summary>
    /// <remarks>
    /// The segment is defined by its start and end points.
    /// A local frame is attached to the force to provide auto projection of its components in the given local frame.
    /// </remarks>
    public sealed class DForce : DVector, IDeepCopy<DForce>
    {
        public DForce(MVector valueInGCS, MPoint startPoint, MPoint endPoint, MFrame localFrameInGCS)
            :base(valueInGCS, startPoint, endPoint, localFrameInGCS)
        {
        }
        public DForce(MVector valueInGCS, MPoint startPoint, MPoint endPoint) :base(valueInGCS, startPoint, endPoint)
        {
        }
        DForce IDeepCopy<DForce>.DeepCopy()
        {
            return new DForce(Value, StartPoint, EndPoint, LocalFrame);
        }
    }

    /// <summary>
    /// Represents a uniform distributed moment (m) over a segment.
    /// </summary>
    /// <remarks>
    /// The segment is defined by its start and end points.
    /// A local frame is attached to the force to provide auto projection of its components in the given local frame.
    /// </remarks>
    public sealed class DMoment : DVector, IDeepCopy<DMoment>
    {
        public DMoment(MVector valueInGCS, MPoint startPoint, MPoint endPoint, MFrame localFrameInGCS)
            : base(valueInGCS, startPoint, endPoint, localFrameInGCS)
        {
        }
        public DMoment(MVector valueInGCS, MPoint startPoint, MPoint endPoint) : base(valueInGCS, startPoint, endPoint)
        {
        }

        DMoment IDeepCopy<DMoment>.DeepCopy()
        {
            return new DMoment(Value, StartPoint, EndPoint, LocalFrame);
        }
    }
}
