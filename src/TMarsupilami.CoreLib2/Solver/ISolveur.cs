using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.CoreLib2
{
    public interface ISolveur
    {
        IModel GetModel();
        void BuildAnalysis(IModel model, ILoadCase loadCase);

        // le dispatcher permet d'associer à chaque element ou contrainte layout le bon 
        void BuildAnalysis(IModel model, ILoadCase loadCase, IDispatcher dispatcher);
    }
}
