using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public struct Force
    {
        public MFrame LocalFrame { get; private set; }
        public MVector ValueInGCS { get; private set; }
        public MVector ValueInLCS { get; private set; }

        public Force(MVector value, MFrame localFrame, bool isGlobal)
        {
            this.LocalFrame = localFrame;

            // Store force vector in GCS and LCS
            if (isGlobal)
            {
                this.ValueInGCS = value;
                this.ValueInLCS = ToLocalCoordinateSystem(value, localFrame);
            }
            else
            {
                this.ValueInGCS = ToGlobalCoordinateSystem(value, localFrame);
                this.ValueInLCS = value;
            }
        }

        public MVector ToLocalCoordinateSystem(MFrame localFrameInGCS)
        {
            return ToLocalCoordinateSystem(ValueInGCS, localFrameInGCS);
        }
        public Tuple<MVector, MVector, MVector> GetGlobalComponents()
        {
            var F1 = new MVector(ValueInGCS.X, 0, 0);
            var F2 = new MVector(0, ValueInGCS.Y, 0);
            var F3 = new MVector(0, 0, ValueInGCS.Z);

            var components = new Tuple<MVector, MVector, MVector>(F1, F2, F3);
            return components;
        }
        public Tuple<MVector, MVector, MVector> GetLocalComponents()
        {
            var F1 = ValueInLCS.X * LocalFrame.XAxis;
            var F2 = ValueInLCS.Y * LocalFrame.YAxis;
            var F3 = ValueInLCS.Z * LocalFrame.ZAxis;

            var components = new Tuple<MVector, MVector, MVector>(F1, F2, F3);
            return components;
        }
        public override string ToString()
        {
            return ValueInGCS.ToString();
        }

        public static MVector ToGlobalCoordinateSystem(MVector valueInLCS, MFrame localFrameInGCS)
        {
            return valueInLCS.X * localFrameInGCS.XAxis
                    + valueInLCS.Y * localFrameInGCS.YAxis
                    + valueInLCS.Z * localFrameInGCS.ZAxis;
        }
        public static MVector ToLocalCoordinateSystem(MVector valueInGCS, MFrame localFrameInGCS)
        {
            return new MVector(valueInGCS * localFrameInGCS.XAxis,
                                valueInGCS * localFrameInGCS.YAxis,
                                valueInGCS * localFrameInGCS.ZAxis
                              );
        }
    }

}
