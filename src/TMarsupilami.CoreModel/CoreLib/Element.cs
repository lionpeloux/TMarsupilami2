using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public abstract class Element
    {
        public Node Nodes
        {
            get => default(Node);
            set
            {
            }
        }
    }
}