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
using System.Text;
using System.Runtime.InteropServices;


namespace FLIR
{
    class CPI
    {
        // CPI Exceptions
        public class Exception : System.Exception
        {
            // strerror
            [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
            private static extern IntPtr cpi_strerror(int error);
            private static string strerror(int error)
            {
                return Marshal.PtrToStringAnsi(cpi_strerror(error));
            }

            public Exception() : base("General Error") { }
            public Exception(int error) : base(strerror(error)) { }
            public Exception(string serror) : base(serror) { }
        }

        // cerial interface
        public class Cerial
        {
            public class Exception : System.Exception
            {
                [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
                private static extern int cerstrerror(IntPtr cer, byte[] path, int len);
                private static string strerror(IntPtr handle)
                {
                    byte[] error = new byte[128];
                    string serror;

                    cerstrerror(handle, error, error.Length);
                    serror = Encoding.ASCII.GetString(error);
                    return serror.Remove(serror.IndexOf('\0'));
                }
                public Exception(IntPtr handle) : base(strerror(handle)) { }
            }

            // handle pointer
            private IntPtr _handle;
            public IntPtr handle {
                get { return _handle; }
            }

            // IOCTL requests
            public enum ioctl
            {
                BAUDRATE_GET,
                BAUDRATE_SET,
                TIMEOUT_SET,
                FLUSH_INPUT
            }

            [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
            private static extern int ceropen(IntPtr cer, string path, int flags);
            [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
            private static extern int cerioctl(IntPtr cer, ioctl request);
            [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
            private static extern int cerioctl(IntPtr cer, ioctl request, out int arg);
            [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
            private static extern int cerclose(IntPtr cer);

            // generic exception wrapper around low-level cerial functions
            private void rcwrap(int rc){
                if(rc != 0){
                    throw new Exception(_handle);
                }
            }

            // set baudrate
            public void Baudrate(int baud)
            {
                rcwrap(cerioctl(_handle, ioctl.BAUDRATE_SET, out baud));
            }

            // get baudrate
            public int Baudrate()
            {
                int baud;

                rcwrap(cerioctl(_handle, ioctl.BAUDRATE_GET, out baud));

                return baud;
            }

            // set timeout
            public void Timeout(int milliseconds)
            {
                rcwrap(cerioctl(_handle, ioctl.TIMEOUT_SET, out milliseconds));
            }

            // flush input
            public void FlushInput()
            {
                rcwrap(cerioctl(_handle, ioctl.FLUSH_INPUT));
            }

            // open with ceropen
            private void open(string path)
            {
                // allocate 16 bytes on the heap for the cerial handle
                // the handle itself should be just two pointers in a structure
                // let's be conservative, so allocate at least two 64 bit pointers worth
                _handle = Marshal.AllocHGlobal(16);
                rcwrap(ceropen(_handle, path, 0));
            }
            public Cerial(string path)
            {
                open(path);
            }
            ~Cerial()
            {
                cerclose(_handle);
                Marshal.FreeHGlobal(_handle);
            }
        }
        
        // cerial handle
        private Cerial cerial;

        // types
        public enum opcode
        {
            OP_INVALID,
            OP_NOOP,
            OP_PAN_DESIRED_POS_GET,
            OP_PAN_DESIRED_POS_SET,
            OP_PAN_DESIRED_POS_REL_SET,
            OP_PAN_CURRENT_POS_GET,
            OP_PAN_DESIRED_SPEED_GET,
            OP_PAN_DESIRED_SPEED_SET,
            OP_PAN_CURRENT_SPEED_GET,
            OP_PAN_REL_SPEED_SET,
            OP_PAN_UPPER_SPEED_LIMIT_GET,
            OP_PAN_UPPER_SPEED_LIMIT_SET,
            OP_PAN_LOWER_SPEED_LIMIT_GET,
            OP_PAN_LOWER_SPEED_LIMIT_SET,
            OP_PAN_ACCEL_GET,
            OP_PAN_ACCEL_SET,
            OP_PAN_BASE_SPEED_GET,
            OP_PAN_BASE_SPEED_SET,
            OP_PAN_HOLD_POWER_GET,
            OP_PAN_HOLD_POWER_SET,
            OP_PAN_MOVE_POWER_GET,
            OP_PAN_MOVE_POWER_SET,
            OP_PAN_MIN_POSITION,
            OP_PAN_MAX_POSITION,
            OP_PAN_USER_MIN_POS_GET,
            OP_PAN_USER_MIN_POS_SET,
            OP_PAN_USER_MAX_POS_GET,
            OP_PAN_USER_MAX_POS_SET,
            OP_PAN_RESOLUTION,
            OP_PAN_CONTINUOUS_GET,
            OP_PAN_CONTINUOUS_SET,
            OP_TILT_DESIRED_POS_GET,
            OP_TILT_DESIRED_POS_SET,
            OP_TILT_DESIRED_POS_REL_SET,
            OP_TILT_CURRENT_POS_GET,
            OP_TILT_DESIRED_SPEED_GET,
            OP_TILT_DESIRED_SPEED_SET,
            OP_TILT_CURRENT_SPEED_GET,
            OP_TILT_REL_SPEED_SET,
            OP_TILT_UPPER_SPEED_LIMIT_GET,
            OP_TILT_UPPER_SPEED_LIMIT_SET,
            OP_TILT_LOWER_SPEED_LIMIT_GET,
            OP_TILT_LOWER_SPEED_LIMIT_SET,
            OP_TILT_ACCEL_GET,
            OP_TILT_ACCEL_SET,
            OP_TILT_BASE_SPEED_GET,
            OP_TILT_BASE_SPEED_SET,
            OP_TILT_HOLD_POWER_GET,
            OP_TILT_HOLD_POWER_SET,
            OP_TILT_MOVE_POWER_GET,
            OP_TILT_MOVE_POWER_SET,
            OP_TILT_MIN_POSITION,
            OP_TILT_MAX_POSITION,
            OP_TILT_USER_MIN_POS_GET,
            OP_TILT_USER_MIN_POS_SET,
            OP_TILT_USER_MAX_POS_GET,
            OP_TILT_USER_MAX_POS_SET,
            OP_TILT_RESOLUTION,
            OP_TILT_CONTINUOUS_GET,
            OP_TILT_CONTINUOUS_SET,
            OP_GEO_POINT_LAST,
            OP_GEO_POINT_LANDMARK,
            OP_GEO_POINT_LLA,
            OP_GEO_CAL,
            OP_RESERVED_AAA,
            OP_RESERVED_AAB,
            OP_GEO_DIST_LANDMARK,
            OP_GEO_DIST_LLA,
            OP_GEO_LLA_GET,
            OP_GEO_LLA_SET,
            OP_GEO_PTU_RPY_GET,
            OP_GEO_PTU_RPY_SET,
            OP_GEO_ALT_GET,
            OP_GEO_ALT_SET,
            OP_GEO_LAT_GET,
            OP_GEO_LAT_SET,
            OP_GEO_LON_GET,
            OP_GEO_LON_SET,
            OP_GEO_PTU_PITCH_GET,
            OP_GEO_PTU_PITCH_SET,
            OP_GEO_PTU_ROLL_GET,
            OP_GEO_PTU_ROLL_SET,
            OP_GEO_PTU_YAW_GET,
            OP_GEO_PTU_YAW_SET,
            OP_GEO_CAM_PITCH_GET,
            OP_GEO_CAM_PITCH_SET,
            OP_GEO_CAL_QUALITY_GET,
            OP_GEO_LAND_PRINT_GET,
            OP_GEO_LAND_IDX_GET,
            OP_GEO_LAND_NUM_GET,
            OP_RESERVED_AAC,
            OP_GEO_LAND_CLEAR,
            OP_GEO_LAND_DEL_IDX,
            OP_GEO_LAND_DEL_LAST,
            OP_GEO_LAND_ADD,
            OP_GEO_DEFAULT_FACTORY,
            OP_GEO_DEFAULT_RESTORE,
            OP_GEO_DEFAULT_SAVE,
            OP_GEO_STATUS_GET,
            OP_GEO_POINT_TYPE_GET,
            OP_GEO_POINT_TYPE_SET,
            OP_ISM_STAB_HEAD_RESUME,
            OP_ISM_CMD_MODE_GET,
            OP_ISM_CMD_MODE_SET,
            OP_ISM_HEAD_PAN_SPD_DES_GET,
            OP_ISM_HEAD_PAN_SPD_DES_SET,
            OP_ISM_HEAD_TILT_SPD_DES_GET,
            OP_ISM_HEAD_TILT_SPD_DES_SET,
            OP_ISM_HEAD_PAN_SPD_CUR_GET,
            OP_ISM_HEAD_PAN_SPD_REL_SET,
            OP_ISM_HEAD_TILT_SPD_CUR_GET,
            OP_ISM_HEAD_TILT_SPD_REL_SET,
            OP_ISM_HEAD_PAN_POS_DES_GET,
            OP_ISM_HEAD_PAN_POS_DES_SET,
            OP_ISM_HEAD_TILT_POS_DES_GET,
            OP_ISM_HEAD_TILT_POS_DES_SET,
            OP_ISM_HEAD_PAN_POS_CUR_GET,
            OP_ISM_HEAD_PAN_POS_REL_SET,
            OP_ISM_HEAD_TILT_POS_CUR_GET,
            OP_ISM_HEAD_TILT_POS_REL_SET,
            OP_ISM_HEAD_PAN_GAIN_GET,
            OP_ISM_HEAD_PAN_GAIN_SET,
            OP_ISM_HEAD_TILT_GAIN_GET,
            OP_ISM_HEAD_TILT_GAIN_SET,
            OP_ISM_STAB_PAN_SPD_DES_GET,
            OP_ISM_STAB_PAN_SPD_DES_SET,
            OP_ISM_STAB_TILT_SPD_DES_GET,
            OP_ISM_STAB_TILT_SPD_DES_SET,
            OP_ISM_STAB_PAN_SPD_CUR_GET,
            OP_ISM_STAB_PAN_SPD_REL_SET,
            OP_ISM_STAB_TILT_SPD_CUR_GET,
            OP_ISM_STAB_TILT_SPD_REL_SET,
            OP_ISM_STAB_PAN_POS_DES_GET,
            OP_ISM_STAB_PAN_POS_DES_SET,
            OP_ISM_STAB_TILT_POS_DES_GET,
            OP_ISM_STAB_TILT_POS_DES_SET,
            OP_ISM_STAB_PAN_POS_CUR_GET,
            OP_ISM_STAB_PAN_POS_REL_SET,
            OP_ISM_STAB_TILT_POS_CUR_GET,
            OP_ISM_STAB_TILT_POS_REL_SET,
            OP_ISM_PAN_KP_GET,
            OP_ISM_PAN_KP_SET,
            OP_ISM_TILT_KP_GET,
            OP_ISM_TILT_KP_SET,
            OP_ISM_PAN_KF_GET,
            OP_ISM_PAN_KF_SET,
            OP_ISM_TILT_KF_GET,
            OP_ISM_TILT_KF_SET,
            OP_ISM_PAN_KI_GET,
            OP_ISM_PAN_KI_SET,
            OP_ISM_TILT_KI_GET,
            OP_ISM_TILT_KI_SET,
            OP_ISM_PAN_KD_GET,
            OP_ISM_PAN_KD_SET,
            OP_ISM_TILT_KD_GET,
            OP_ISM_TILT_KD_SET,
            OP_ISM_PAN_KS_GET,
            OP_ISM_PAN_KS_SET,
            OP_ISM_TILT_KS_GET,
            OP_ISM_TILT_KS_SET,
            OP_ISM_PAN_KV_GET,
            OP_ISM_PAN_KV_SET,
            OP_ISM_TILT_KV_GET,
            OP_ISM_TILT_KV_SET,
            OP_ISM_PAN_KA_GET,
            OP_ISM_PAN_KA_SET,
            OP_ISM_TILT_KA_GET,
            OP_ISM_TILT_KA_SET,
            OP_ISM_PAN_VD_GET,
            OP_ISM_PAN_VD_SET,
            OP_ISM_TILT_VD_GET,
            OP_ISM_TILT_VD_SET,
            OP_ISM_PAN_PD_GET,
            OP_ISM_PAN_PD_SET,
            OP_ISM_TILT_PD_GET,
            OP_ISM_TILT_PD_SET,
            OP_ISM_PAN_IW_GET,
            OP_ISM_PAN_IW_SET,
            OP_ISM_TILT_IW_GET,
            OP_ISM_TILT_IW_SET,
            OP_ISM_X_BIAS_GET,
            OP_ISM_X_BIAS_SET,
            OP_ISM_Y_BIAS_GET,
            OP_ISM_Y_BIAS_SET,
            OP_ISM_Z_BIAS_GET,
            OP_ISM_Z_BIAS_SET,
            OP_ISM_DEFAULT_SAVE,
            OP_ISM_DEFAULT_FACTORY,
            OP_ISM_DEFAULT_RESTORE,
            OP_ISM_STAB_EN,
            OP_ISM_STAB_DIS,
            OP_ISM_GYRO_CAL,
            OP_ISM_GYRO_UTC_GET,
            OP_ISM_GYRO_UTC_SET,
            OP_ISM_LOS_MODE_GET,
            OP_ISM_LOS_MODE_SET,
            OP_ISM_GYRO_UPGRADE,
            OP_ISM_GYRO_FIRMWARE_STATUS_GET,
            OP_RESERVED_AAD,
            OP_RESERVED_AAE,
            OP_ISM_STATUS_GET,
            OP_RESERVED_AAF,
            OP_RESERVED_AAG,
            OP_ISM_RESET_UP_NORTH,
            OP_ISM_CAM_PITCH_OFFSET_GET,
            OP_ISM_CAM_PITCH_OFFSET_SET,
            OP_CONTROL_TYPE_GET,
            OP_CONTROL_TYPE_SET,
            OP_CONTROL_PAN_CORRECTIONS_GET,
            OP_CONTROL_TILT_CORRECTIONS_GET,
            OP_TIMESTAMP_COUNT_GET,
            OP_TIMESTAMP_FREQUENCY,
            OP_EXEC_MODE_GET,
            OP_EXEC_MODE_SET,
            OP_AWAIT,
            OP_MOTION_GET,
            OP_MOTION_SET,
            OP_MOTION_TIMESTAMP_GET,
            OP_SPEED_CONTROL_MODE_GET,
            OP_SPEED_CONTROL_MODE_SET,
            OP_ECHO_GET,
            OP_ECHO_SET,
            OP_FEEDBACK_GET,
            OP_FEEDBACK_SET,
            OP_HALT,
            OP_JOYSTICK_GET,
            OP_LIMIT_GET,
            OP_LIMIT_SET,
            OP_MONITOR_DEFAULT,
            OP_MONITOR_PAN,
            OP_MONITOR_ALL,
            OP_MONITOR_ENABLE,
            OP_MONITOR_GET,
            OP_MONITOR_STATUS_GET,
            OP_ENVIRON_GET,
            OP_RESET,
            OP_RESET_TYPE,
            OP_UNIT_ID_GET,
            OP_UNIT_ID_SET,
            OP_SW_VERSION_GET,
            OP_SW_VERSION_SHORT_GET,
            OP_COMPAT_GET,
            OP_COMPAT_SET,
            OP_DEFAULT_FACTORY,
            OP_DEFAULT_RESTORE,
            OP_DEFAULT_SAVE,
            OP_NET_MAC_GET,
            OP_NET_MAC_SET,
            OP_NET_FIREWALL_PUSH,
            OP_NET_FIREWALL_POP,
            OP_NET_FIREWALL_FLUSH,
            OP_NET_FIREWALL_COUNT,
            OP_NET_FIREWALL_INDEX_GET,
            OP_NET_FIREWALL_INDEX_SET,
            OP_NET_FIREWALL_APPLY,
            OP_NET_GATEWAY_GET,
            OP_NET_GATEWAY_SET,
            OP_NET_IP_GET,
            OP_NET_IP_SET,
            OP_NET_MODE_GET,
            OP_NET_MODE_SET,
            OP_NET_HOSTNAME_GET,
            OP_NET_HOSTNAME_SET,
            OP_NET_NETMASK_GET,
            OP_NET_NETMASK_SET,
            OP_PELCO_PARSE_GET,
            OP_PELCO_PARSE_SET,
            OP_PELCO_ADDRESS_GET,
            OP_PELCO_ADDRESS_SET,
            OP_PAN_STEP_GET,
            OP_PAN_STEP_SET,
            OP_TILT_STEP_GET,
            OP_TILT_STEP_SET,
            OP_SERIAL_GET,
            OP_MODEL_GET,
            OP_PRESET_GO,
            OP_PRESET_SET,
            OP_PRESET_CLEAR,
            OP_OPIO_OUT,
            OP_RESERVED_AAH,
            OP_RESERVED_AAI,
            OP_RESERVED_AAJ,
            OP_RESERVED_AAK,
            OP_RESERVED_AAL,
            OP_RESERVED_AAM,
            OP_RESERVED_AAN,
            OP_RESERVED_AAO,
            OP_RESERVED_AAP,
            OP_RESERVED_AAQ,
            OP_RESERVED_AAR,
            OP_RESERVED_AAS,
            OP_RESERVED_AAT,
            OP_RESERVED_AAU,
            OP_RESERVED_AAV,
            OP_RESERVED_AAW,
            OP_RESERVED_AAX,
            OP_RESERVED_AAY,
            OP_RESERVED_AAZ,
            OP_RESERVED_ABA,
            OP_RESERVED_ABB,
            OP_RESERVED_ABC,
            OP_RESERVED_ABD,
            OP_RESERVED_ABE,
            OP_RESERVED_ABF,
            OP_RESERVED_ABG,
            OP_RESERVED_ABH,
            OP_RESERVED_ABI,
            OP_RESERVED_ABJ,
            OP_RESERVED_ABK,
            OP_RESERVED_ABL,
            OP_RESERVED_ABM,
            OP_RESERVED_ABN,
            OP_RESERVED_ABO,
            OP_RESERVED_ABP,
            OP_RESERVED_ABQ,
            OP_RESERVED_ABR,
            OP_RESERVED_ABS,
            OP_RESERVED_ABT,
            OP_HOST_SETTINGS_SET,
            OP_CHA_SETTINGS_SET,
            OP_CHB_SETTINGS_SET,
            OP_CH_RX,
            OP_CH_TX,
            OP_RESERVED_ABU,
            OP_NET_REDIRECT_GET,
            OP_NET_REDIRECT_SET
        };

        // Enumerations
        public enum Enable { DISABLE, ENABLE }
        public enum MotorPower { HIGH, REGULAR, LOW, OFF };
        public enum StepMode { FULL, HALF, QUARTER, EIGHTH, AUTO };
        public enum Channel { HOST, CHA, CHB };
        public enum Control { CONTROL_INDEPENDENT = 1, CONTROL_PURE_VELOCITY };
        public enum Feedback { ASCII_FEEDBACK_TERSE, ASCII_FEEDBACK_VERBOSE };
        public enum Limits { LIMITS_FACTORY, LIMITS_USER, LIMITS_DISABLED };
        public enum IpMode { NET_IP_STATIC, NET_IP_DYNAMIC };
        public enum OpioOut { OPIO_OUT_LOW, OPIO_OUT_HIGH };
        public enum ExecutionType { IMMEDIATE_MODE, SLAVE_MODE };
        public enum ResetType
        {
            RESET_DEFAULT,
            RESET_NONE,
            RESET_ALL,
            RESET_PAN,
            RESET_TILT
        };
        public enum Halt
        {
            HALT_ALL,
            HALT_PAN,
            HALT_TILT
        };

        // get the status from the last PTCmd execution
        UInt16 status;
        public UInt16 Status()
        {
            return status;
        }

        // generic exception wrapper around low-level cpi functions
        private static void rcwrap(int rc){
            if(rc != 0){
                throw new Exception(rc);
            }
        }

        // resync
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_resync(IntPtr cer);
        public void Resync(){ rcwrap(cpi_resync(cerial.handle)); }

        // ptcmd imports
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, Enable a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, Int32 a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, Int32 a, Int32 b);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, Int32 a, Int32 b, Int32 c, Int32 d);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, UInt32 a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, UInt32 a, UInt32 b);

        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, UInt32 a, out double b);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, double a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, double a, double b, double c);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, double a, double b, double c, out double d);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out Int32 a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out Int32 a, out Int32 b, out Int32 c, out Int32 d);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out Int32 a, out Int32 b, out Int32 c, out Int32 d, out UInt32 e);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out UInt32 a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out double a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out double a, out Int32 b, out Int32 c, out Int32 d);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, out double a, out double b, out double c);

        public void PTCmd(opcode op) { rcwrap(cpi_ptcmd(cerial.handle, out status, op)); }
        public void PTCmd(opcode op, Int32 a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a)); }
        public void PTCmd(opcode op, Int32 a, Int32 b) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, b)); }
        public void PTCmd(opcode op, Int32 a, Int32 b, Int32 c, Int32 d) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, b, c, d)); }
        public void PTCmd(opcode op, UInt32 a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a)); }
        public void PTCmd(opcode op, UInt32 a, UInt32 b) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, b)); }
        public void PTCmd(opcode op, UInt32 a, out double b) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, out b)); }
        public void PTCmd(opcode op, double a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a)); }
        public void PTCmd(opcode op, double a, double b, double c) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, b, c)); }
        public void PTCmd(opcode op, double a, double b, double c, out double d) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, b, c, out d)); }
        public void PTCmd(opcode op, out Int32 a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a)); }
        public void PTCmd(opcode op, out Int32 a, out Int32 b, out Int32 c, out Int32 d) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a, out b, out c, out d)); }
        public void PTCmd(opcode op, out Int32 a, out Int32 b, out Int32 c, out Int32 d, out UInt32 e) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a, out b, out c, out d, out e)); }
        public void PTCmd(opcode op, out UInt32 a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a)); }
        public void PTCmd(opcode op, out double a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a)); }
        public void PTCmd(opcode op, out double a, out Int32 b, out Int32 c, out Int32 d) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a, out b, out c, out d)); }
        public void PTCmd(opcode op, out double a, out double b, out double c) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, out a, out b, out c)); }
        
        public void PTCmd(opcode op, Enable a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, (int)a)); }
        public void PTCmd(opcode op, MotorPower a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, (int)a)); }
        public void PTCmd(opcode op, StepMode a) { rcwrap(cpi_ptcmd(cerial.handle, out status, op, (int)a)); }
        public void PTCmd(opcode op, out Enable a)
        {
            int i;
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, out i));
            a = (Enable)i;
        }
        public void PTCmd(opcode op, out MotorPower a)
        {
            int i;
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, out i));
            a = (MotorPower)i;
        }
        public void PTCmd(opcode op, out StepMode a)
        {
            int i;
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, out i));
            a = (StepMode)i;
        }

        // String methods are special. We must wrap them because safe C strings have more than one argument per string.
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, int len, out int rxlen, byte[] str);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, Int32 a, int len, out int rxlen, byte[] str);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, int len, string a);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, Int32 a, int blen, string b);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, int alen, string a, double b, double c, double d);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, UInt32 a, UInt32 b, int clen, string c);
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int cpi_ptcmd(IntPtr cer, out UInt16 status, opcode op, UInt32 a, UInt32 b, int clen, string c, int dlen, string d);
        
        // helper to wrap receiving strings
        private delegate int cpi_OutStrCall(byte[] buff, out int rxlen);
        private void PTCmdOutStr(out string str, cpi_OutStrCall call)
        {
            byte[] buff = new byte[128];
            int rxlen;

            rcwrap(call(buff, out rxlen));

            str = Encoding.ASCII.GetString(buff, 0, rxlen);
        }

        // PTCmds with strings
        public void PTCmd(opcode op, out string str)
        {
            PTCmdOutStr(out str, delegate(byte[] buff, out int rxlen) { return cpi_ptcmd(cerial.handle, out status, op, buff.Length, out rxlen, buff); });
        }
        public void PTCmd(opcode op, Int32 a, out string str)
        {
            PTCmdOutStr(out str, delegate(byte[] buff, out int rxlen) { return cpi_ptcmd(cerial.handle, out status, op, a, buff.Length, out rxlen, buff); });
        }
        public void PTCmd(opcode op, string str)
        {
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, str.Length, str));
        }
        public void PTCmd(opcode op, Int32 a, string str)
        {
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, str.Length, str));
        }
        public void PTCmd(opcode op, string str, double a, double b, double c)
        {
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, str.Length, str, a, b, c));
        }
        public void PTCmd(opcode op, UInt32 a, UInt32 b, string c, string d)
        {
            rcwrap(cpi_ptcmd(cerial.handle, out status, op, a, b, c.Length, c, d.Length, d));
        }

        // version string
        [DllImport("libcpi.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr cpi_version_str();
        public static string VersionString()
        {
            return Marshal.PtrToStringAnsi(cpi_version_str());
        }

        public CPI(string path)
        {
            cerial = new Cerial(path);
            Resync();
        }
    }
}
