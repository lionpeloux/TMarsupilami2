using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore
{
    interface IDeepCloneable<T>
    {
        /// <summary>
        /// Deep Copy.
        /// </summary>
        /// <returns>A (deep) copy of the object, of the same type.</returns>
        T DeepClone();
    }
}
