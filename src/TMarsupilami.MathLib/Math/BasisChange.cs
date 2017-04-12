using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class BasisChange
    {

        public static MVector ToGlobal(MVector valueInLCS, MFrame localFrameInGCS)
        {
            return valueInLCS.X * localFrameInGCS.XAxis
                    + valueInLCS.Y * localFrameInGCS.YAxis
                    + valueInLCS.Z * localFrameInGCS.ZAxis;
        }

        public static MVector ToLocal(MVector valueInGCS, MFrame localFrameInGCS)
        {
            return new MVector(valueInGCS * localFrameInGCS.XAxis,
                                valueInGCS * localFrameInGCS.YAxis,
                                valueInGCS * localFrameInGCS.ZAxis
                              );
        }
    }
}
