using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public struct Torsor : IDeepCopy<Torsor>, IShallowCopy<Torsor>
    {
        public MFrame LocalFrame { get; private set; }
        public MVector Force { get; private set; }
        public MVector Moment { get; private set; }

        public Torsor(MVector forceInGCS, MVector momentInGCS, MFrame localFrameInGCS)
        {
            LocalFrame = localFrameInGCS;
            Force = forceInGCS;
            Moment = momentInGCS;
        }
        public Torsor(MVector forceInGCS, MVector momentInGCS, MPoint applicationPoint)
        {
            var localFrame = MFrame.XY;
            localFrame.Origin = applicationPoint;
            LocalFrame = localFrame;
            Force = forceInGCS;
            Moment = momentInGCS;
        }
        public Torsor(MFrame localFrameInGCS) : this(MVector.Zero, MVector.Zero, localFrameInGCS)
        {
        }
        public Torsor(MPoint applicationPoint) : this(MVector.Zero, MVector.Zero, applicationPoint)
        {
        }

        public Torsor Move(MPoint toPoint)
        {
            var localFrame = LocalFrame;
            localFrame.Origin = toPoint;

            var u = new MVector(LocalFrame.Origin, localFrame.Origin);
            var force = Force;
            var moment = Moment + MVector.CrossProduct(Force, u);

            return new Torsor(force, moment, localFrame);
        }
        public Torsor Move(MFrame toFrame)
        {
            var localFrame = toFrame;

            var u = new MVector(LocalFrame.Origin, localFrame.Origin);
            var force = Force;
            var moment = Moment + MVector.CrossProduct(Force, u);

            return new Torsor(force, moment, localFrame);
        }

        public void GetForceComponents(out MVector F1, out MVector F2, out MVector F3, bool inGCS)
        {
            if (inGCS)
            {
                F1 = new MVector(Force.X, 0, 0);
                F2 = new MVector(0, Force.Y, 0);
                F3 = new MVector(0, 0, Force.Z);
            }
            else
            {
                GetForceComponents(LocalFrame, out F1, out F2, out F3);
            }

        }
        public void GetMomentComponents(out MVector M1, out MVector M2, out MVector M3, bool inGCS)
        {
            if (inGCS)
            {
                M1 = new MVector(Moment.X, 0, 0);
                M2 = new MVector(0, Moment.Y, 0);
                M3 = new MVector(0, 0, Moment.Z);
            }
            else
            {
                GetMomentComponents(LocalFrame, out M1, out M2, out M3);
            }
        }

        public void GetForceComponents(MFrame localFrameInGCS, out MVector F1, out MVector F2, out MVector F3)
        {
            var forceInLCS = BasisChange.ToLocal(Force, localFrameInGCS);
            F1 = forceInLCS.X * localFrameInGCS.XAxis;
            F2 = forceInLCS.Y * localFrameInGCS.YAxis;
            F3 = forceInLCS.Z * localFrameInGCS.ZAxis;
        }
        public void GetMomentComponents(MFrame localFrameInGCS, out MVector M1, out MVector M2, out MVector M3)
        {
            var momentInLCS = BasisChange.ToLocal(Moment, localFrameInGCS);
            M1 = momentInLCS.X * localFrameInGCS.XAxis;
            M2 = momentInLCS.Y * localFrameInGCS.YAxis;
            M3 = momentInLCS.Z * localFrameInGCS.ZAxis;
        }

        public void GetForceCoordinates(out double F1, out double F2, out double F3, bool inGCS)
        {
            if (inGCS)
            {
                F1 = Force.X;
                F2 = Force.Y;
                F3 = Force.Z;
            }
            else
            {
                GetForceCoordinates(LocalFrame, out F1, out F2, out F3);
            }
        }
        public void GetMomentCoordinates(out double M1, out double M2, out double M3, bool inGCS)
        {
            if (inGCS)
            {
                M1 = Moment.X;
                M2 = Moment.Y;
                M3 = Moment.Z;
            }
            else
            {
                GetMomentCoordinates(LocalFrame, out M1, out M2, out M3);
            }
        }

        public void GetForceCoordinates(MFrame localFrameInGCS, out double F1, out double F2, out double F3)
        {
            var forceInLCS = BasisChange.ToLocal(Force, localFrameInGCS);
            F1 = forceInLCS.X;
            F2 = forceInLCS.Y;
            F3 = forceInLCS.Z;
        }
        public void GetMomentCoordinates(MFrame localFrameInGCS, out double M1, out double M2, out double M3)
        {
            var momentInLCS = BasisChange.ToLocal(Moment, localFrameInGCS);
            M1 = momentInLCS.X;
            M2 = momentInLCS.Y;
            M3 = momentInLCS.Z;
        }
        public override string ToString()
        {
            return "[T] = { O : " + LocalFrame.Origin + " | F : " + Force + " | M : " + Moment + " }";
        }

        public Torsor DeepCopy()
        {
            return new Torsor(Force, Moment, LocalFrame);
        }
        public Torsor ShallowCopy()
        {
            return DeepCopy();
        }
    }


}
