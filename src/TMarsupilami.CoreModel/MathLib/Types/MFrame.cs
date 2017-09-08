using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TMarsupilami.CoreModel
{
    public struct MFrame
    {
        private MPoint origin;
        private MVector xaxis;
        private MVector yaxis;
        private MVector zaxis;

        public MPoint Origin
        {
            get => origin;
            set
            {
                origin = value;
            }
        }

        public MVector XAxis
        {
            get => default(MVector);
            set
            {
            }
        }

        public MVector YAxis
        {
            get => default(MVector);
            set
            {
            }
        }

        public MVector ZAxis
        {
            get => default(MVector);
            set
            {
            }
        }
    }
}