using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    
    public class CVector : IDeepCopy<CVector>
    {
        public MFrame LocalFrame { get; private set; }
        public MVector Value { get; private set; }

        public CVector(MVector valueInGCS, MFrame localFrameInGCS)
        {
            this.LocalFrame = localFrameInGCS;
            this.Value = valueInGCS;
        }
        public CVector(MVector valueInGCS) :this(valueInGCS, MFrame.XY)
        {
        }

        public void GetComponents(out MVector V1, out MVector V2, out MVector V3, bool inGCS)
        {
            if (inGCS)
            {
                V1 = new MVector(Value.X, 0, 0);
                V2 = new MVector(0, Value.Y, 0);
                V3 = new MVector(0, 0, Value.Z);
            }
            else
            {
                GetComponents(LocalFrame, out V1, out V2, out V3);
            }

        }
        public void GetComponents(MFrame localFrameInGCS, out MVector V1, out MVector V2, out MVector V3)
        {
            var valueInLCS = BasisChange.ToLocal(Value, localFrameInGCS);
            V1 = valueInLCS.X * localFrameInGCS.XAxis;
            V2 = valueInLCS.Y * localFrameInGCS.YAxis;
            V3 = valueInLCS.Z * localFrameInGCS.ZAxis;
        }

        public void GetCoordinates(out double V1, out double V2, out double V3, bool inGCS)
        {
            if (inGCS)
            {
                V1 = Value.X;
                V2 = Value.Y;
                V3 = Value.Z;
            }
            else
            {
                GetCoordinates(LocalFrame, out V1, out V2, out V3);
            }
        }
        public void GetCoordinates(MFrame localFrameInGCS, out double V1, out double V2, out double V3)
        {
            var valueInLCS = BasisChange.ToLocal(Value, localFrameInGCS);
            V1 = valueInLCS.X;
            V2 = valueInLCS.Y;
            V3 = valueInLCS.Z;
        }

        public override string ToString()
        {
            return "[T] = { O : " + LocalFrame.Origin + " | V : " + Value + " }";
        }
        public CVector DeepCopy()
        {
            return new CVector(Value, LocalFrame);
        }
    }

    /// <summary>
    /// Represents a concentrated force (F).
    /// </summary>
    /// <remarks>
    /// A local frame is attached to the force vector to provide auto projection of its components in the given local frame.
    /// The local frame origin is assumed to be the location of the force vector for display.
    /// </remarks>
    public sealed class CForce : CVector, IDeepCopy<CForce>
    {
        public CForce(MVector valueInGCS, MFrame localFrameInGCS)
            :base(valueInGCS, localFrameInGCS)
        {
        }
        public CForce(MVector valueInGCS) :base(valueInGCS)
        {
        }

        CForce IDeepCopy<CForce>.DeepCopy()
        {
            return new CForce(Value, LocalFrame);
        }
    }

    /// <summary>
    /// Represents a concentrated moment (M).
    /// </summary>
    /// <remarks>
    /// A local frame is attached to the moment vector to provide auto projection of its components in the given local frame.
    /// The local frame origin is assumed to be the location of the moment vector for display.
    /// </remarks>
    public sealed class CMoment : CVector, IDeepCopy<CMoment>
    {
        public CMoment(MVector valueInGCS, MFrame localFrameInGCS)
            : base(valueInGCS, localFrameInGCS)
        {
        }
        public CMoment(MVector valueInGCS) : base(valueInGCS)
        {
        }

        CMoment IDeepCopy<CMoment>.DeepCopy()
        {
            return new CMoment(Value, LocalFrame);
        }
    }
}
