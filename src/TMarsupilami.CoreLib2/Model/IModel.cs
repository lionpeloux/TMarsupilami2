using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.CoreLib2
{
    // Description Performantiel du model
    public interface IModel
    {
        bool AddElement(IElementLayout element);
        bool AddConstraint(IConstraintLayout constraint);
    }
}
