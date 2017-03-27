using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.CoreLib2
{
    public interface IShallowCopy<T>
    {
        T ShallowCopy();
    }
}
