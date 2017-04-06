using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public sealed class Force
    {
        public bool IsGlobal { get; private set; }
        public MFrame LocalFrame { get; private set; }
        public MVector F { get; private set; }

        public Force(MVector force, MFrame localFrame, bool isGlobal)
        {
            this.F = force;
            this.LocalFrame = localFrame;
            this.IsGlobal = isGlobal;
        }

        public MVector ToGlobalCS()
        {
            if (IsGlobal)
            {
                return F;
            }
            else
            {
                return (F.X * LocalFrame.XAxis + F.Y * LocalFrame.YAxis + F.Z * LocalFrame.ZAxis);
            }
        }
        public MVector ToLocalCS()
        {
            if (!IsGlobal)
            {
                return F;
            }
            else
            {
                return new MVector(F * LocalFrame.XAxis, F * LocalFrame.YAxis, F * LocalFrame.ZAxis);
            }
        }
        public override string ToString()
        {
            if (IsGlobal)
            {
                return String.Format("GCS : " + F);
            }
            else
            {
                return String.Format("LCS : " + F);
            }
        }
    }

}
