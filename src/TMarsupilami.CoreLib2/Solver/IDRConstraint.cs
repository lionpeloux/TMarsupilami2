using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib2
{
    /// <summary>
    /// Tuyauterie pour faire fonctionner une contrainte dans un solveur de relaxation dynamique
    /// </summary>
    public interface IDRConstraint
    {
        void Init();
        void Enforce_Mr();
        void Enforce_Qr();
        void Enforce_Fr();
    }
    

}
