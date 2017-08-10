using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public class Node3Dof : Node
    {
        private MPoint point;
        private MVector vx;
        private MVector ax;
        private MVector F;

        public virtual MPoint Position { get { return point; } set { point = value; } }
        public MVector Velocity { get { return vx; } set { vx = value; } }
        public MVector Acceleration { get { return ax; } set { ax = value; } }
        public MVector Force { get { return F; } set { F = value; } }
    }
}