using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public interface ISurface
    {
        MPoint ClosestPoint(MPoint targetPoint);
        double Distance(MPoint targetPoint);
    }
}