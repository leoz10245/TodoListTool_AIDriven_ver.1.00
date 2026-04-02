using System;
using System.Collections.Generic;

namespace MotorController;

/// <summary>
/// Hardware Abstraction Layer (HAL) interface used by the motor controller core.
/// This corresponds to the "hal にスタブ HAL 実装" described in Template.nproj.
/// </summary>
public interface IHardwareAbstractionLayer
{
    double ReadAdVoltage();

    void SetPowerPortEnabled(bool enabled);

    void SendCan(string channel, string message);

    bool TryReceiveCan(string channel, out string message);

    void SendSerial(string message);

    bool TryReceiveSerial(out string message);

    DateTime UtcNow { get; }

    void PersistNvmSnapshot(IReadOnlyCollection<string> dtcs);
}

/// <summary>
/// Simple stub implementation of the HAL. It only logs operations to the console
/// and keeps minimal in-memory state so that the higher-level logic can run.
/// </summary>
public sealed class StubHal : IHardwareAbstractionLayer
{
    private double _voltage = 12.0;
    private readonly List<string> _nvmSnapshots = new();

    public double ReadAdVoltage() => _voltage;

    /// <summary>
    /// Allows tests or demos to adjust the simulated supply voltage.
    /// </summary>
    public void SetSimulatedVoltage(double voltage) => _voltage = voltage;

    public void SetPowerPortEnabled(bool enabled)
        => Console.WriteLine($"[HAL] Power port {(enabled ? "ON" : "OFF")}");

    public void SendCan(string channel, string message)
        => Console.WriteLine($"[CAN:{channel}] {message}");

    public bool TryReceiveCan(string channel, out string message)
    {
        // No real bus in this demo; nothing is received.
        message = string.Empty;
        return false;
    }

    public void SendSerial(string message)
        => Console.WriteLine($"[SERIAL] {message}");

    public bool TryReceiveSerial(out string message)
    {
        message = string.Empty;
        return false;
    }

    public DateTime UtcNow => DateTime.UtcNow;

    public void PersistNvmSnapshot(IReadOnlyCollection<string> dtcs)
    {
        var snapshot = string.Join(",", dtcs);
        _nvmSnapshots.Add(snapshot);
        Console.WriteLine($"[NVM] Snapshot saved: {snapshot}");
    }
}

/// <summary>
/// Core runtime for the motor control software.
/// Implements the behavior described in Template.nproj:
/// - task_init, task_5ms, task_10ms, task_100ms
/// - Init_Run, CanComm_Run, SerialComm_Run, TempMgmt_Run,
///   RpmCtrl_Run, PowerMgmt_Run, Diag_Run, Refresh_Run
/// </summary>
public sealed class MotorControllerRuntime
{
    private readonly IHardwareAbstractionLayer _hal;
    private readonly State _state = new();

    private const int MaxRpm = 3000;
    private const double LowVoltageThreshold = 9.0;       // 低電圧判定閾値
    private const double OverheatThreshold = 120.0;       // 過熱判定閾値 [degC]
    private const double OverheatRecoverThreshold = 110.0;// ヒステリシス復帰閾値 [degC]
    private static readonly TimeSpan NvmBackupInterval = TimeSpan.FromMilliseconds(500);

    public MotorControllerRuntime(IHardwareAbstractionLayer hal)
    {
        _hal = hal ?? throw new ArgumentNullException(nameof(hal));
    }

    // ---------------------------------------------------------------------
    // main 相当の周期タスク群
    // ---------------------------------------------------------------------

    /// <summary>
    /// main 相当の初期化タスク (task_init)。
    /// </summary>
    public void task_init()
    {
        Init_Run();
    }

    /// <summary>
    /// 5ms 周期タスク (task_5ms)。主に通信処理を担当させる。
    /// </summary>
    public void task_5ms()
    {
        CanComm_Run();
        SerialComm_Run();
    }

    /// <summary>
    /// 10ms 周期タスク (task_10ms)。温度・回転数・電源制御を行う。
    /// </summary>
    public void task_10ms()
    {
        TempMgmt_Run();
        RpmCtrl_Run();
        PowerMgmt_Run();
    }

    /// <summary>
    /// 100ms 周期タスク (task_100ms)。診断およびリフレッシュ処理を行う。
    /// </summary>
    public void task_100ms()
    {
        Diag_Run();
        Refresh_Run();
    }

    // ---------------------------------------------------------------------
    // Init_Run: サブシステム初期化
    // ---------------------------------------------------------------------

    /// <summary>
    /// 温度管理、回転数制御、電源管理、診断、CAN 通信、シリアル通信の順で初期化する。
    /// </summary>
    public void Init_Run()
    {
        InitializeTemperatureManagement();
        InitializeRpmControl();
        InitializePowerManagement();
        InitializeDiagnostics();
        InitializeCanCommunication();
        InitializeSerialCommunication();
    }

    private void InitializeTemperatureManagement()
    {
        _state.TemperatureCelsius = 25.0;
        Console.WriteLine("[Init] Temperature management initialized.");
    }

    private void InitializeRpmControl()
    {
        _state.TargetRpm = 0;
        Console.WriteLine("[Init] RPM control initialized.");
    }

    private void InitializePowerManagement()
    {
        _state.LastMeasuredVoltage = _hal.ReadAdVoltage();
        _state.ConsecutiveSameVoltageSamples = 1;
        _state.VoltageIsValid = true;
        Console.WriteLine($"[Init] Power management initialized. Voltage={_state.LastMeasuredVoltage:F1}V");
    }

    private void InitializeDiagnostics()
    {
        _state.Dtcs.Clear();
        _state.LastNvmBackupTime = _hal.UtcNow;
        Console.WriteLine("[Init] Diagnostics initialized.");
    }

    private void InitializeCanCommunication()
    {
        Console.WriteLine("[Init] CAN communication initialized.");
    }

    private void InitializeSerialCommunication()
    {
        Console.WriteLine("[Init] Serial communication initialized.");
    }

    // ---------------------------------------------------------------------
    // CanComm_Run: CAN 通信処理
    // ---------------------------------------------------------------------

    /// <summary>
    /// Alive 通知送信、Alive 応答確認、モータ制御要求受信、モータ状態送信、
    /// 診断要求受信、診断応答送信、IG ON タイマ更新を行う。
    /// </summary>
    public void CanComm_Run()
    {
        SendAliveNotification();
        CheckAliveResponse();
        ReceiveMotorControlRequest();
        SendMotorStatus();
        ReceiveDiagnosticRequest();
        SendDiagnosticResponse();
        UpdateIgOnTimer();
    }

    private void SendAliveNotification()
    {
        _hal.SendCan("CTRL", "ALIVE");
    }

    private void CheckAliveResponse()
    {
        if (_hal.TryReceiveCan("CTRL_ACK", out var msg))
        {
            Console.WriteLine($"[CAN] Alive response: {msg}");
            _state.LastAliveAckTime = _hal.UtcNow;
        }
    }

    private void ReceiveMotorControlRequest()
    {
        if (_hal.TryReceiveCan("REQ", out var msg) && int.TryParse(msg, out var rpm))
        {
            _state.RequestedRpmFromCan = rpm;
            _state.HasPowerRequest = rpm > 0;
        }
    }

    private void SendMotorStatus()
    {
        var payload = $"RPM={_state.TargetRpm},Temp={_state.TemperatureCelsius:F1},LV={_state.LowVoltageFlag}";
        _hal.SendCan("STATUS", payload);
    }

    private void ReceiveDiagnosticRequest()
    {
        if (_hal.TryReceiveCan("DIAG_REQ", out var msg))
        {
            _state.DiagRequest = msg;
        }
    }

    private void SendDiagnosticResponse()
    {
        if (_state.DiagRequest is null)
        {
            return;
        }

        string response = _state.DiagRequest switch
        {
            "READ_DTC" => string.Join(",", _state.Dtcs),
            "CLEAR_ALL" => "CLEARED",
            _ => "UNKNOWN_REQ"
        };

        if (_state.DiagRequest == "CLEAR_ALL")
        {
            _state.Dtcs.Clear();
        }

        _hal.SendCan("DIAG_RES", response);
        _state.DiagRequest = null;
    }

    private void UpdateIgOnTimer()
    {
        _state.IgOnTimerMs += 5; // called from 5ms task
    }

    // ---------------------------------------------------------------------
    // SerialComm_Run: シリアル通信処理
    // ---------------------------------------------------------------------

    /// <summary>
    /// 電源ポートの OFF→ON 遷移検出、電源投入直後マスク管理、回転数指令送信、
    /// 温度受信、受信タイムアウト診断を行う。
    /// </summary>
    public void SerialComm_Run()
    {
        DetectPowerPortOnTransition();
        ManageInitialMaskAfterPowerOn();
        SendRpmCommandOverSerial();
        ReceiveTemperatureOverSerial();
        PerformReceiveTimeoutDiagnosis();
    }

    private void DetectPowerPortOnTransition()
    {
        bool previous = _state.PreviousPowerPortOn;
        bool current = _state.PowerPortOn;
        if (!previous && current)
        {
            _state.PowerOnTimestamp = _hal.UtcNow;
            Console.WriteLine("[Serial] Power port OFF→ON detected.");
        }

        _state.PreviousPowerPortOn = current;
    }

    private void ManageInitialMaskAfterPowerOn()
    {
        if (_state.PowerOnTimestamp is null)
        {
            return;
        }

        var elapsed = _hal.UtcNow - _state.PowerOnTimestamp.Value;
        _state.IsInitialMaskActive = elapsed < TimeSpan.FromSeconds(1);
    }

    private void SendRpmCommandOverSerial()
    {
        _hal.SendSerial($"RPM_CMD={_state.TargetRpm}");
    }

    private void ReceiveTemperatureOverSerial()
    {
        if (_hal.TryReceiveSerial(out var msg) &&
            msg.StartsWith("TEMP=", StringComparison.OrdinalIgnoreCase) &&
            double.TryParse(msg.AsSpan("TEMP=".Length), out var temp))
        {
            _state.TemperatureCelsius = temp;
            _state.LastTempReceiveTime = _hal.UtcNow;
        }
    }

    private void PerformReceiveTimeoutDiagnosis()
    {
        var now = _hal.UtcNow;
        if (now - _state.LastTempReceiveTime > TimeSpan.FromSeconds(1))
        {
            RecordDtcOnce("TEMP_TIMEOUT");
        }
    }

    // ---------------------------------------------------------------------
    // TempMgmt_Run: 温度管理
    // ---------------------------------------------------------------------

    /// <summary>
    /// 温度 CAN 反映、過熱判定、ヒステリシスによる復帰判定、過熱 DTC 記録を行う。
    /// </summary>
    public void TempMgmt_Run()
    {
        ReflectTemperatureToCan();
        EvaluateOverheat();
        EvaluateOverheatRecoveryWithHysteresis();
        RecordOverheatDtcIfNeeded();
    }

    private void ReflectTemperatureToCan()
    {
        _hal.SendCan("TEMP", _state.TemperatureCelsius.ToString("F1"));
    }

    private void EvaluateOverheat()
    {
        if (_state.TemperatureCelsius >= OverheatThreshold)
        {
            _state.IsOverheated = true;
        }
    }

    private void EvaluateOverheatRecoveryWithHysteresis()
    {
        if (_state.IsOverheated && _state.TemperatureCelsius <= OverheatRecoverThreshold)
        {
            _state.IsOverheated = false;
        }
    }

    private void RecordOverheatDtcIfNeeded()
    {
        if (_state.IsOverheated)
        {
            RecordDtcOnce("OVERHEAT");
        }
    }

    // ---------------------------------------------------------------------
    // RpmCtrl_Run: 回転数制御
    // ---------------------------------------------------------------------

    /// <summary>
    /// 電源要求、温度、低電圧フラグ、電圧妥当性を条件に回転数指令を計算し、
    /// CAN 要求値を 25rpm 刻みへ変換して上限制限する。
    /// </summary>
    public void RpmCtrl_Run()
    {
        bool canDrive =
            _state.HasPowerRequest &&
            !_state.IsOverheated &&
            !_state.LowVoltageFlag &&
            _state.VoltageIsValid;

        int target = canDrive ? _state.RequestedRpmFromCan : 0;
        target = QuantizeRpm(target);
        target = Math.Clamp(target, 0, MaxRpm);

        _state.TargetRpm = target;
    }

    private static int QuantizeRpm(int rpm)
        => (rpm / 25) * 25;

    // ---------------------------------------------------------------------
    // PowerMgmt_Run: 電源管理
    // ---------------------------------------------------------------------

    /// <summary>
    /// AD 電圧読取り、同値 2 回確認フィルタ、CAN 反映、低電圧判定、電源ポート制御、
    /// 電圧 DTC 記録を行う。
    /// </summary>
    public void PowerMgmt_Run()
    {
        double voltage = _hal.ReadAdVoltage();

        if (Math.Abs(voltage - _state.LastMeasuredVoltage) < 0.01)
        {
            _state.ConsecutiveSameVoltageSamples++;
        }
        else
        {
            _state.ConsecutiveSameVoltageSamples = 1;
            _state.LastMeasuredVoltage = voltage;
        }

        _state.VoltageIsValid = _state.ConsecutiveSameVoltageSamples >= 2;
        _state.LowVoltageFlag = _state.VoltageIsValid && voltage < LowVoltageThreshold;

        ReflectVoltageToCan(voltage);
        ControlPowerPort(voltage);
        RecordVoltageDtcIfNeeded(voltage);
    }

    private void ReflectVoltageToCan(double voltage)
    {
        _hal.SendCan("VBAT", voltage.ToString("F2"));
    }

    private void ControlPowerPort(double voltage)
    {
        bool shouldEnablePort = voltage >= LowVoltageThreshold;
        _state.PowerPortOn = shouldEnablePort;
        _hal.SetPowerPortEnabled(shouldEnablePort);
    }

    private void RecordVoltageDtcIfNeeded(double voltage)
    {
        if (_state.LowVoltageFlag)
        {
            RecordDtcOnce("LOW_VOLTAGE");
        }
    }

    // ---------------------------------------------------------------------
    // Diag_Run: 診断
    // ---------------------------------------------------------------------

    /// <summary>
    /// DTC 読出しおよび全消去要求への応答は CanComm_Run 内で取り扱い、
    /// ここでは 500ms ごとの NVM バックアップを行う。
    /// </summary>
    public void Diag_Run()
    {
        BackupNvmPeriodically();
    }

    private void BackupNvmPeriodically()
    {
        var now = _hal.UtcNow;
        if (now - _state.LastNvmBackupTime >= NvmBackupInterval)
        {
            _hal.PersistNvmSnapshot(_state.Dtcs);
            _state.LastNvmBackupTime = now;
        }
    }

    // ---------------------------------------------------------------------
    // Refresh_Run: リフレッシュ
    // ---------------------------------------------------------------------

    /// <summary>
    /// CAN、シリアル、電源ポート方向の再設定を行う。
    /// 実機ではレジスタ再設定などを行うが、ここではログのみを出力する。
    /// </summary>
    public void Refresh_Run()
    {
        Console.WriteLine("[Refresh] Reconfiguring CAN, serial, and power port directions.");
    }

    // ---------------------------------------------------------------------
    // 内部ユーティリティ
    // ---------------------------------------------------------------------

    private void RecordDtcOnce(string code)
    {
        if (!_state.Dtcs.Contains(code))
        {
            _state.Dtcs.Add(code);
        }
    }

    private sealed class State
    {
        // 要求・回転数
        public bool HasPowerRequest;
        public int RequestedRpmFromCan;
        public int TargetRpm;

        // 温度管理
        public double TemperatureCelsius = 25.0;
        public bool IsOverheated;
        public DateTime LastTempReceiveTime;

        // 電源管理
        public bool LowVoltageFlag;
        public bool VoltageIsValid;
        public double LastMeasuredVoltage;
        public int ConsecutiveSameVoltageSamples;
        public bool PowerPortOn;
        public bool PreviousPowerPortOn;
        public DateTime? PowerOnTimestamp;
        public bool IsInitialMaskActive;

        // タイマ
        public int IgOnTimerMs;
        public DateTime LastAliveAckTime;
        public DateTime LastNvmBackupTime;

        // 診断
        public string? DiagRequest;
        public List<string> Dtcs { get; } = new();
    }
}
