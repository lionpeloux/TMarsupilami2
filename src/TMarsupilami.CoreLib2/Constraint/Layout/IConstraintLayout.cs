using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.CoreLib2
{
    public interface IConstraintLayout
    {
    }

    public interface IAppliedDisplacement : IConstraintLayout
    {
    }

    public interface IJoint : IConstraintLayout
    {
    }

    public interface IBoundaryCondition : IConstraintLayout
    {
    }

}
