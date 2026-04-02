using System;

namespace MotorController;

internal static class Program
{
    private static void Main(string[] args)
    {
        IHardwareAbstractionLayer hal = new StubHal();
        var runtime = new MotorControllerRuntime(hal);

        // main相当: 初期化と周期タスクを簡易に実行する
        runtime.task_init();

        // 5ms周期タスクを20回分シミュレーションし、その中で10ms/100msタスクを呼び出す
        for (var i = 0; i < 20; i++)
        {
            runtime.task_5ms();

            if (i % 2 == 0)
            {
                runtime.task_10ms();
            }

            if (i % 20 == 0)
            {
                runtime.task_100ms();
            }
        }

        Console.WriteLine("MotorController run completed.");
    }
}
