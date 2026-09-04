#include "VofaTuning.h"

#include "Config/CarConfig.h"
#include "CarControl.h"
#include "CarMotionLink.h"
#include "DebugPrint.h"
#include "LineFollow.h"
#include "OLED.h"
#include "Serial.h"
#include "SpeedPI.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum
{
    VOFA_PLOT_OFF = 0,
    VOFA_PLOT_SPEED = 1,
    VOFA_PLOT_LINE = 2
} VofaPlotMode_t;

static char g_commandBuffer[VOFA_COMMAND_BUFFER_SIZE];
static uint16_t g_commandLength;
static bool g_commandOverflow;
static VofaPlotMode_t g_plotMode;
static uint32_t g_plotPeriodMs;
static uint32_t g_lastPlotMs;

static char ToLower(char value)
{
    return ((value >= 'A') && (value <= 'Z')) ?
        (char)(value - 'A' + 'a') : value;
}

static char *Normalize(char *text)
{
    char *start;
    char *end;
    char *cursor;

    if (text == 0)
    {
        return 0;
    }

    start = text;
    while ((*start == ' ') || (*start == '\t'))
    {
        start++;
    }

    end = start + strlen(start);
    while ((end > start) && ((end[-1] == ' ') || (end[-1] == '\t')))
    {
        end--;
    }
    *end = '\0';

    for (cursor = start; *cursor != '\0'; cursor++)
    {
        *cursor = ToLower(*cursor);
    }
    return start;
}

/* Accept integers and VOFA-style integer values such as 200.000000. */
static bool ParseI32(const char *text, int32_t *result)
{
    const char *cursor;
    uint32_t value = 0U;
    uint32_t limit;
    bool negative = false;
    bool hasDigit = false;

    if ((text == 0) || (result == 0) || (*text == '\0'))
    {
        return false;
    }

    cursor = text;
    if ((*cursor == '+') || (*cursor == '-'))
    {
        negative = (*cursor == '-');
        cursor++;
    }

    limit = negative ? 2147483648U : 2147483647U;
    while ((*cursor >= '0') && (*cursor <= '9'))
    {
        uint32_t digit = (uint32_t)(*cursor - '0');
        hasDigit = true;
        if (value > ((limit - digit) / 10U))
        {
            return false;
        }
        value = value * 10U + digit;
        cursor++;
    }

    if (!hasDigit)
    {
        return false;
    }

    if (*cursor == '.')
    {
        cursor++;
        while ((*cursor >= '0') && (*cursor <= '9'))
        {
            if (*cursor != '0')
            {
                return false;
            }
            cursor++;
        }
    }

    if (*cursor != '\0')
    {
        return false;
    }

    if (negative)
    {
        *result = (value == 2147483648U) ? INT32_MIN : -(int32_t)value;
    }
    else
    {
        *result = (int32_t)value;
    }
    return true;
}

static bool ParseAssignment(
    const char *command,
    const char *name,
    int32_t *value)
{
    size_t length = strlen(name);
    return (strncmp(command, name, length) == 0) &&
        (command[length] == '=') &&
        ParseI32(&command[length + 1U], value);
}

static void SendError(const char *reason)
{
    Serial_SendString("ERR:");
    Serial_SendString(reason);
    Serial_SendByte((uint8_t)'\n');
}

static void SendOk(const char *name, int32_t value)
{
    Serial_SendString("OK:");
    DebugPrint_NamedI32(name, value);
    Serial_SendByte((uint8_t)'\n');
}

static void SendParameters(void)
{
    CarControlStatus_t car = CarControl_GetStatus();
    SpeedPIGains_t m1 = SpeedPI_GetGains(SPEED_PI_MOTOR1);
    SpeedPIGains_t m2 = SpeedPI_GetGains(SPEED_PI_MOTOR2);
    int32_t lineKp;
    int32_t lineKd;

    LineFollow_GetGains(&lineKp, &lineKd);
    Serial_SendString("PARAM:");
    DebugPrint_NamedI32("base", car.baseCps);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("lkp", lineKp);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("lkd", lineKd);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("m1kp", m1.kpQ12);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("m1ki", m1.kiQ12);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("m2kp", m2.kpQ12);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("m2ki", m2.kiQ12);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("mission", (int32_t)car.missionMode);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("lap", car.lapCompleted ? 1 : 0);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("straight", CarControl_IsStraightTuning() ? 1 : 0);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("plot", (int32_t)g_plotMode);
    Serial_SendByte((uint8_t)',');
    DebugPrint_NamedI32("period", (int32_t)g_plotPeriodMs);
    Serial_SendByte((uint8_t)'\n');
}

static void SendHelp(void)
{
    Serial_SendString(
        "HELP:get | start | start2 | start4 | start56 | start456 | fast | req4 | balance | stop | reset\n"
        "HELP:mission=2(stop A) / 4,56,456(shared continuous 1500 CPS) | straight[=N]\n"
        "HELP:base=N | lkp=N | lkd=N\n"
        "HELP:m1kp=N | m1ki=N | m2kp=N | m2ki=N\n"
        "HELP:skp=N | ski=N\n"
        "HELP:plot=0(off)/1(speed)/2(line) | period=20..1000\n"
        "HELP:car | line | speed | time | oled | oledreset | motionlink\n"
    );
}

static bool SetLineKp(int32_t value)
{
    int32_t kp;
    int32_t kd;
    LineFollow_GetGains(&kp, &kd);
    return LineFollow_SetGains(value, kd);
}

static bool SetLineKd(int32_t value)
{
    int32_t kp;
    int32_t kd;
    LineFollow_GetGains(&kp, &kd);
    return LineFollow_SetGains(kp, value);
}

static bool SetSpeedGain(SpeedPIWheel_t wheel, bool setKp, int32_t value)
{
    SpeedPIGains_t gains = SpeedPI_GetGains(wheel);
    if (setKp)
    {
        gains.kpQ12 = value;
    }
    else
    {
        gains.kiQ12 = value;
    }
    return SpeedPI_SetGains(wheel, gains.kpQ12, gains.kiQ12);
}

static bool SetBothKp(int32_t value)
{
    SpeedPIGains_t m1 = SpeedPI_GetGains(SPEED_PI_MOTOR1);
    SpeedPIGains_t m2 = SpeedPI_GetGains(SPEED_PI_MOTOR2);

    if ((value < SPEED_PI_KP_Q12_MIN) || (value > SPEED_PI_KP_Q12_MAX))
    {
        return false;
    }
    return SpeedPI_SetGains(SPEED_PI_MOTOR1, value, m1.kiQ12) &&
        SpeedPI_SetGains(SPEED_PI_MOTOR2, value, m2.kiQ12);
}

static bool SetBothKi(int32_t value)
{
    SpeedPIGains_t m1 = SpeedPI_GetGains(SPEED_PI_MOTOR1);
    SpeedPIGains_t m2 = SpeedPI_GetGains(SPEED_PI_MOTOR2);

    if ((value < SPEED_PI_KI_Q12_MIN) || (value > SPEED_PI_KI_Q12_MAX))
    {
        return false;
    }
    return SpeedPI_SetGains(SPEED_PI_MOTOR1, m1.kpQ12, value) &&
        SpeedPI_SetGains(SPEED_PI_MOTOR2, m2.kpQ12, value);
}

static void ProcessCommand(char *rawCommand, uint32_t nowMs)
{
    char *command = Normalize(rawCommand);
    int32_t value;

    if ((command == 0) || (*command == '\0'))
    {
        return;
    }

    if (strcmp(command, "get") == 0)
    {
        SendParameters();
        return;
    }
    if (strcmp(command, "help") == 0)
    {
        SendHelp();
        return;
    }
    if (strcmp(command, "fast") == 0)
    {
        if (!CarControl_SetMissionMode(CAR_MISSION_STOP_AT_A))
        {
            SendError("mission_change_rejected");
        }
        else
        {
            SendOk("mission", 2);
        }
        return;
    }
    if ((strcmp(command, "req45") == 0) ||
        (strcmp(command, "req4") == 0) ||
        (strcmp(command, "balance") == 0))
    {
        if (!CarControl_SetMissionMode(CAR_MISSION_BALANCE_CENTER))
        {
            SendError("mission_change_rejected");
        }
        else
        {
            SendOk("mission", 45);
        }
        return;
    }
    if (strcmp(command, "req6") == 0)
    {
        if (!CarControl_SetMissionMode(CAR_MISSION_BALANCE_CAPTURED_TARGET))
        {
            SendError("mission_change_rejected");
        }
        else
        {
            SendOk("mission", 6);
        }
        return;
    }
    if (strcmp(command, "start2") == 0)
    {
        if (!CarControl_StartMode(nowMs, CAR_MISSION_STOP_AT_A))
        {
            SendError("start2_rejected");
        }
        else
        {
            Serial_SendString("OK:start2\n");
        }
        return;
    }
    if ((strcmp(command, "start45") == 0) ||
        (strcmp(command, "start4") == 0) ||
        (strcmp(command, "start5") == 0) ||
        (strcmp(command, "start56") == 0) ||
        (strcmp(command, "start456") == 0))
    {
        if (!CarControl_StartMode(nowMs, CAR_MISSION_BALANCE_CENTER))
        {
            SendError("start45_rejected");
        }
        else
        {
            Serial_SendString("OK:start45_center\n");
        }
        return;
    }
    if (strcmp(command, "start6") == 0)
    {
        if (!CarControl_StartMode(
                nowMs,
                CAR_MISSION_BALANCE_CAPTURED_TARGET))
        {
            SendError("start6_rejected");
        }
        else
        {
            Serial_SendString("OK:start6_capture_current\n");
        }
        return;
    }
    if (strcmp(command, "start") == 0)
    {
        if (!CarControl_Start(nowMs))
        {
            SendError("start_rejected");
        }
        else
        {
            Serial_SendString("OK:start\n");
        }
        return;
    }
    if (strcmp(command, "straight") == 0)
    {
        value = CarControl_GetStatus().baseCps;
        if (!CarControl_StartStraightTuning(nowMs, value))
        {
            SendError("straight_start_rejected");
        }
        else
        {
            SendOk("straight", value);
        }
        return;
    }
    if (strcmp(command, "stop") == 0)
    {
        (void)CarControl_Stop(nowMs);
        Serial_SendString("OK:stop\n");
        return;
    }
    if (strcmp(command, "reset") == 0)
    {
        if (!CarControl_Reset(nowMs))
        {
            SendError("reset_rejected");
        }
        else
        {
            Serial_SendString("OK:reset\n");
        }
        return;
    }
    if (strcmp(command, "car") == 0)
    {
        CarControl_PrintStatus();
        return;
    }
    if (strcmp(command, "line") == 0)
    {
        LineFollow_PrintStatus();
        return;
    }
    if (strcmp(command, "speed") == 0)
    {
        SpeedPI_PrintStatus();
        return;
    }

    if (strcmp(command, "time") == 0)
    {
        Serial_SendString("TIME:");
        DebugPrint_U32(CarControl_GetDriveTimeMs());
        Serial_SendString("ms running=");
        DebugPrint_Bool(CarControl_IsDriveTimerRunning());
        Serial_SendByte((uint8_t)'\n');
        return;
    }
    if (strcmp(command, "oled") == 0)
    {
        OLEDDiagnostics_t diagnostics;

        OLED_GetDiagnostics(&diagnostics);
        Serial_SendString("OLED:online=");
        DebugPrint_Bool(diagnostics.online != 0U);
        Serial_SendString(" attempts=");
        DebugPrint_U32(diagnostics.connectAttempts);
        Serial_SendString(" connectFail=");
        DebugPrint_U32(diagnostics.connectFailures);
        Serial_SendString(" runtimeFail=");
        DebugPrint_U32(diagnostics.runtimeFailures);
        Serial_SendByte((uint8_t)'\n');
        return;
    }
    if (strcmp(command, "oledreset") == 0)
    {
        CarState_t state = CarControl_GetStatus().state;

        if ((state == CAR_STATE_PRETILT) ||
            (state == CAR_STATE_RUNNING) ||
            (state == CAR_STATE_PASSING_FINISH) ||
            (state == CAR_STATE_STOPPING))
        {
            SendError("oledreset_rejected_while_motors_active");
        }
        else if (OLED_ForceReconnect(nowMs))
        {
            Serial_SendString("OK:oled_reconnected\n");
        }
        else
        {
            SendError("oled_reconnect_failed");
        }
        return;
    }
    if (strcmp(command, "motionlink") == 0)
    {
        CarMotionLinkStatus_t link = CarMotionLink_GetStatus();

        Serial_SendString("MOTION_TX sent=");
        DebugPrint_U32(link.sentFrameCount);
        Serial_SendString(" dropped=");
        DebugPrint_U32(link.droppedFrameCount);
        Serial_SendString(" bytes=");
        DebugPrint_U32(link.sentByteCount);
        Serial_SendString(" seq=");
        DebugPrint_U32(link.sequence);
        Serial_SendString(" busy/index=");
        DebugPrint_Bool(link.txBusy != 0U);
        Serial_SendByte((uint8_t)'/');
        DebugPrint_U32(link.txIndex);
        Serial_SendByte((uint8_t)'\n');

        Serial_SendString("BALANCE_ACK valid=");
        DebugPrint_Bool(link.ackValid != 0U);
        Serial_SendString(" mission=");
        DebugPrint_U32(link.ackMission);
        Serial_SendString(" status=");
        DebugPrint_Mask8(link.ackStatus);
        Serial_SendString(" pretiltReady=");
        DebugPrint_Bool(
            (link.ackStatus & BALANCE_ACK_PRETILT_READY) != 0U);
        Serial_SendString(" error=");
        DebugPrint_U32(link.ackError);
        Serial_SendString(" age=");
        DebugPrint_U32(link.ackAgeMs);
        Serial_SendString("ms frames/checksum/bytes=");
        DebugPrint_U32(link.ackFrameCount);
        Serial_SendByte((uint8_t)'/');
        DebugPrint_U32(link.ackChecksumErrorCount);
        Serial_SendByte((uint8_t)'/');
        DebugPrint_U32(link.ackByteCount);
        Serial_SendByte((uint8_t)'\n');
        return;
    }
    if (ParseAssignment(command, "straight", &value))
    {
        if (!CarControl_StartStraightTuning(nowMs, value))
        {
            SendError("straight_start_rejected");
        }
        else
        {
            SendOk("straight", value);
        }
        return;
    }
    if (ParseAssignment(command, "mission", &value))
    {
        CarMissionMode_t mode;
        if (value == 2)
        {
            mode = CAR_MISSION_STOP_AT_A;
        }
        else if ((value == 4) || (value == 5) ||
                 (value == 45) || (value == 456))
        {
            mode = CAR_MISSION_BALANCE_CENTER;
        }
        else if (value == 6)
        {
            mode = CAR_MISSION_BALANCE_CAPTURED_TARGET;
        }
        else
        {
            SendError("mission_must_be_2_45_or_6");
            return;
        }

        if (!CarControl_SetMissionMode(mode))
        {
            SendError("mission_change_rejected");
        }
        else
        {
            SendOk("mission", value);
        }
        return;
    }
    if (ParseAssignment(command, "base", &value))
    {
        if (!CarControl_SetBaseCps(value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("base", value);
        }
        return;
    }
    if (ParseAssignment(command, "lkp", &value))
    {
        if (!SetLineKp(value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("lkp", value);
        }
        return;
    }
    if (ParseAssignment(command, "lkd", &value))
    {
        if (!SetLineKd(value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("lkd", value);
        }
        return;
    }
    if (ParseAssignment(command, "m1kp", &value))
    {
        if (!SetSpeedGain(SPEED_PI_MOTOR1, true, value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("m1kp", value);
        }
        return;
    }
    if (ParseAssignment(command, "m1ki", &value))
    {
        if (!SetSpeedGain(SPEED_PI_MOTOR1, false, value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("m1ki", value);
        }
        return;
    }
    if (ParseAssignment(command, "m2kp", &value))
    {
        if (!SetSpeedGain(SPEED_PI_MOTOR2, true, value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("m2kp", value);
        }
        return;
    }
    if (ParseAssignment(command, "m2ki", &value))
    {
        if (!SetSpeedGain(SPEED_PI_MOTOR2, false, value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("m2ki", value);
        }
        return;
    }
    if (ParseAssignment(command, "skp", &value))
    {
        if (!SetBothKp(value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("skp", value);
        }
        return;
    }
    if (ParseAssignment(command, "ski", &value))
    {
        if (!SetBothKi(value))
        {
            SendError("out_of_range");
        }
        else
        {
            SendOk("ski", value);
        }
        return;
    }
    if (ParseAssignment(command, "plot", &value))
    {
        if ((value < (int32_t)VOFA_PLOT_OFF) ||
            (value > (int32_t)VOFA_PLOT_LINE))
        {
            SendError("plot_must_be_0_1_2");
        }
        else
        {
            g_plotMode = (VofaPlotMode_t)value;
            g_lastPlotMs = nowMs;
            SendOk("plot", value);
        }
        return;
    }
    if (ParseAssignment(command, "period", &value))
    {
        if ((value < (int32_t)VOFA_MIN_PLOT_PERIOD_MS) ||
            (value > (int32_t)VOFA_MAX_PLOT_PERIOD_MS))
        {
            SendError("out_of_range");
        }
        else
        {
            g_plotPeriodMs = (uint32_t)value;
            g_lastPlotMs = nowMs;
            SendOk("period", value);
        }
        return;
    }

    SendError("unknown_command");
}

static void ProcessRx(uint32_t nowMs)
{
    uint8_t data;
    uint32_t received = 0U;

    /*
     * Bound debug command work in one main-loop pass. A noisy adapter or a
     * continuous host stream must not starve line following or UART1 service.
     */
    while ((received < 64U) && Serial_TryReadByte(&data))
    {
        received++;
        if (data == (uint8_t)'\r')
        {
            continue;
        }

        if (data == (uint8_t)'\n')
        {
            if (g_commandOverflow)
            {
                SendError("command_too_long");
            }
            else if (g_commandLength > 0U)
            {
                g_commandBuffer[g_commandLength] = '\0';
                ProcessCommand(g_commandBuffer, nowMs);
            }
            g_commandLength = 0U;
            g_commandOverflow = false;
            continue;
        }

        if (g_commandOverflow)
        {
            continue;
        }

        if (g_commandLength >= (VOFA_COMMAND_BUFFER_SIZE - 1U))
        {
            g_commandLength = 0U;
            g_commandOverflow = true;
            continue;
        }

        g_commandBuffer[g_commandLength++] = (char)data;
    }
}

static void SendCsvValue(int32_t value, bool comma)
{
    DebugPrint_I32(value, false);
    if (comma)
    {
        Serial_SendByte((uint8_t)',');
    }
}

static void SendSpeedFrame(void)
{
    SpeedPIStatus_t speed = SpeedPI_GetStatus();

    /* M1: target,raw,filtered,error,P,I,output,sat; then the same for M2. */
    Serial_SendString("speed:");
    SendCsvValue(speed.motor1.targetCps, true);
    SendCsvValue(speed.motor1.rawCps, true);
    SendCsvValue(speed.motor1.filteredCps, true);
    SendCsvValue(speed.motor1.errorCps, true);
    SendCsvValue(speed.motor1.proportionalPercent, true);
    SendCsvValue(speed.motor1.integralPercent, true);
    SendCsvValue(speed.motor1.outputPercent, true);
    SendCsvValue(speed.motor1.saturated ? 1 : 0, true);
    SendCsvValue(speed.motor2.targetCps, true);
    SendCsvValue(speed.motor2.rawCps, true);
    SendCsvValue(speed.motor2.filteredCps, true);
    SendCsvValue(speed.motor2.errorCps, true);
    SendCsvValue(speed.motor2.proportionalPercent, true);
    SendCsvValue(speed.motor2.integralPercent, true);
    SendCsvValue(speed.motor2.outputPercent, true);
    SendCsvValue(speed.motor2.saturated ? 1 : 0, false);
    Serial_SendByte((uint8_t)'\n');
}

static void SendLineFrame(void)
{
    CarControlStatus_t car = CarControl_GetStatus();
    LineFollowStatus_t line = LineFollow_GetStatus();
    SpeedPIStatus_t speed = SpeedPI_GetStatus();

    Serial_SendString("line:");
    SendCsvValue(line.error, true);
    SendCsvValue(line.derivative, true);
    SendCsvValue(line.correctionCps, true);
    SendCsvValue(line.leftTargetCps, true);
    SendCsvValue(line.rightTargetCps, true);
    SendCsvValue(speed.motor1.filteredCps, true);
    SendCsvValue(speed.motor2.filteredCps, true);
    SendCsvValue((int32_t)car.sensorMask, true);
    SendCsvValue(car.lineValid ? 1 : 0, true);
    SendCsvValue((int32_t)car.linePattern, true);
    SendCsvValue((int32_t)car.state, true);
    SendCsvValue(line.replayingLastState ? 1 : 0, true);
    SendCsvValue((int32_t)line.lineLostElapsedMs, false);
    Serial_SendByte((uint8_t)'\n');
}

static void PlotTask(uint32_t nowMs)
{
    if (g_plotMode == VOFA_PLOT_OFF)
    {
        return;
    }

    if ((uint32_t)(nowMs - g_lastPlotMs) < g_plotPeriodMs)
    {
        return;
    }

    g_lastPlotMs += g_plotPeriodMs;
    if ((uint32_t)(nowMs - g_lastPlotMs) >= g_plotPeriodMs)
    {
        g_lastPlotMs = nowMs;
    }

    if (g_plotMode == VOFA_PLOT_SPEED)
    {
        SendSpeedFrame();
    }
    else
    {
        SendLineFrame();
    }
}

void VofaTuning_Init(uint32_t nowMs)
{
    g_commandLength = 0U;
    g_commandOverflow = false;
    g_plotMode = VOFA_PLOT_OFF;
    g_plotPeriodMs = VOFA_DEFAULT_PLOT_PERIOD_MS;
    g_lastPlotMs = nowMs;

    Serial_SendString("VOFA_READY:115200,FireWater\n");
    Serial_SendString(
        "VOFA_SPEED:m1Target,m1Raw,m1Filtered,m1Error,m1P,m1I,m1Pwm,m1Sat,"
        "m2Target,m2Raw,m2Filtered,m2Error,m2P,m2I,m2Pwm,m2Sat\n"
    );
    Serial_SendString(
        "VOFA_LINE:error,derivative,correction,leftTarget,rightTarget,"
        "m1Filtered,m2Filtered,mask,lineValid,pattern,state,replay,lostMs\n"
    );
    SendParameters();
}

void VofaTuning_Task(uint32_t nowMs)
{
    ProcessRx(nowMs);
    PlotTask(nowMs);
}
