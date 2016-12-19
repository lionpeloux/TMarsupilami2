using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore
{
    public struct Frame
    {
        private Point origin;
        private Vector t;
        private Vector d1;
        private Vector d2;

        public Point Origin { get; set; }
        public Vector T { get; set; }
        public Vector D1 { get; set; }
        public Vector D2 { get; set; }
    }
}
