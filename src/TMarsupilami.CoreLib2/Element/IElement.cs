using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.CoreLib2
{
    public enum ElementModel : int
    {
        Beam_3,
        Beam_4_Continuous,
        Beam_4_Discontinuous,
    }

    public interface IElement
    {
        //ElementModel Model { get; }
    }
}
