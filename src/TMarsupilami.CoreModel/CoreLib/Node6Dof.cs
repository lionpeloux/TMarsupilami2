using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public class Node6Dof : Node3Dof
    {
        private MFrame orientation;
        private MVector vθ;
        private MVector aθ;
        private MVector M;

        public override MPoint Position { get { return orientation.Origin; } set { orientation.Origin = value; } }
        public MFrame Orientation { get { return orientation; } set { orientation = value; } }

        public MVector AngularVelocity { get { return vθ; } set { vθ = value; } }
        public MVector AngularAcceleration { get { return aθ; } set { aθ = value; } }
        public MVector Moment { get { return M; } set { M = value; } }

    }
}