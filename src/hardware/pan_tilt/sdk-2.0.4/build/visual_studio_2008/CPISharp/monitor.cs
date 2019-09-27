//****************************************************************************
//***              (C)2014 FLIR Commercial Systems, Inc.                 *****
//***                       All Rights Reserved.                         *****
//***                                                                    *****
//***     This source data and code (the "Code") may NOT be distributed  *****
//***     without the express prior written permission consent from of   *****
//***     FLIR Commercial Systems, Inc. ("FLIR").  FLIR PROVIDES THIS    *****
//***     CODE ON AN "AS IS" BASIS FOR USE BY RECIPIENT AT ITS OWN       *****
//***     RISK.  FLIR DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, IMPLIED *****
//***     OR STATUTORY, INCLUDING WITHOUT LIMITATION ANY IMPLIED         *****
//***     WARRANTIES OF TITLE, NON-INFRINGEMENT OF THIRD PARTY RIGHTS,   *****
//***     MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.          *****
//***     FLIR Commercial Systems, Inc. reserves the right to make       *****
//***     changes without further notice to the Code or any content      *****
//***     herein including to improve reliability, function or design.   *****
//***     FLIR Commercial Systems, Inc. shall not assume any liability   *****
//***     arising from the application or use of this code, data or      *****
//***     function.                                                      *****
//***                                                                    *****
//***     FLIR Commercial Systems, Inc.                                  *****
//***     Motion Control Systems                                         *****
//***     www.flir.com/mcs                                               *****
//***     mcs-support@flir.com                                           *****
//****************************************************************************

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace FLIR
{
    class Monitor
    {
        CPI cpi;
        struct PTU
        {
            public int pn, px, tn, tx, pu, tu;
        };
        PTU ptu;

        private void get_info(){
            string model, version;
            int serial;

            // get PTU details
            cpi.PTCmd(CPI.opcode.OP_MODEL_GET, out model);
            cpi.PTCmd(CPI.opcode.OP_SERIAL_GET, out serial);
            cpi.PTCmd(CPI.opcode.OP_SW_VERSION_SHORT_GET, out version);
            Console.WriteLine("SDK v{0} connected to {1} #{2} running v{3}",
                CPI.VersionString(), model, serial, version);
        }

        private void setup()
        {
            // get min/max position and upper speed limits
            cpi.PTCmd(CPI.opcode.OP_PAN_MIN_POSITION, out ptu.pn);
            cpi.PTCmd(CPI.opcode.OP_PAN_MAX_POSITION, out ptu.px);
            cpi.PTCmd(CPI.opcode.OP_TILT_MIN_POSITION, out ptu.tn);
            cpi.PTCmd(CPI.opcode.OP_TILT_MAX_POSITION, out ptu.tx);
            cpi.PTCmd(CPI.opcode.OP_PAN_UPPER_SPEED_LIMIT_GET, out ptu.pu);
            cpi.PTCmd(CPI.opcode.OP_TILT_UPPER_SPEED_LIMIT_GET, out ptu.tu);

            // set desired speed to our maximum speed
            cpi.PTCmd(CPI.opcode.OP_PAN_DESIRED_SPEED_SET, ptu.pu);
            cpi.PTCmd(CPI.opcode.OP_TILT_DESIRED_SPEED_SET, ptu.tu);
        }

        // poll loop for integer returning opcode to return "expected"
        private void wait_int(CPI.opcode op, int expected)
        {
            int current;
            do
            {
                cpi.PTCmd(op, out current);
                if (current != expected)
                {
                    Thread.Sleep(33);
                }
            } while (current != expected);
        }

        private void go_to(int pan, int tilt)
        {
            Console.WriteLine("Going to ({0}, {1})", pan, tilt);

            // command unit to move
            cpi.PTCmd(CPI.opcode.OP_PAN_DESIRED_POS_SET, pan);
            cpi.PTCmd(CPI.opcode.OP_TILT_DESIRED_POS_SET, tilt);

            // wait for it to arrive
            wait_int(CPI.opcode.OP_PAN_CURRENT_POS_GET, pan);
            wait_int(CPI.opcode.OP_TILT_CURRENT_POS_GET, tilt);
        }

        private void move(){
            // cycle between min/max
            while(true)
            {
                go_to(ptu.pn, ptu.tn);
                go_to(ptu.px, ptu.tx);
            }
        }

        // start monitoring forever
        public void go()
        {
            get_info();
            setup();
            move();
        }

        public Monitor(string CerialPath)
        {
            cpi = new CPI(CerialPath);
        }
    }

    class MonitorMain {
        static void Main(string[] args)
        {
            if (args.Length < 1)
            {
                Console.WriteLine("Usage: monitor <port>");
                Environment.Exit(1);
            }
            try
            {
                Monitor mon = new Monitor(args[0]);
                mon.go();
            }
            catch (CPI.Exception e)
            {
                Console.WriteLine("CPI Error: {0}", e.ToString());
            }
            catch (CPI.Cerial.Exception e)
            {
                Console.WriteLine("Cerial Error: {0}", e.ToString());
            }
            catch (Exception e)
            {
                Console.WriteLine("General exception: {0}", e.ToString());
            }
        }
    }
}
