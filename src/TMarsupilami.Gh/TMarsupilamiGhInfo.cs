using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace TMarsupilamiGh
{
    public class TMarsupilamiGhInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "TMarsupilami";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("6c0576e0-2b0f-4d25-8ce5-353faab6ae1d");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "Copyright © Lionel dU Peloux 2017";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "lionel.dupeloux@gmail.com";
            }
        }
    }
}
