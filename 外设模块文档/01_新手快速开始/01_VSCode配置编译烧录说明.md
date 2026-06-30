# VS Code 红线和开发环境说明

## 为什么 VS Code 里会出现红色报错

如果命令行能正常编译：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_user_gcc.ps1
```

并且显示：

```text
BUILD_OK: firmware.elf + firmware.hex
```

那 VS Code 里的很多红线通常不是固件真的写错，而是代码分析插件没有拿到正确配置。

MSPM0 工程需要这些信息：

- ARM GCC 编译器路径
- `User`
- `User/Inc`
- `Source`
- `Source/third_party/CMSIS/Core/Include`
- `__MSPM0G3507__`
- `DeviceFamily_MSPM0G350X`
- `CONFIG_MSPM0G350X`

如果插件不知道这些路径和宏，就会把下面这些正常符号标红：

- `uint32_t`
- `uint8_t`
- `DL_ADC12_CLOCK_SYSOSC`
- `DL_GPIO_PIN_0`
- `CPUCLK_FREQ`
- `__NOP`

## 已经添加的 VS Code 配置

当前工程已经补了这些文件：

| 文件 | 作用 |
|---|---|
| `.vscode/c_cpp_properties.json` | 给 Microsoft C/C++ 插件提供 include 路径、宏、编译器 |
| `.vscode/settings.json` | 设置默认 C/C++ IntelliSense 配置 |
| `compile_flags.txt` | 给 clangd 这类插件提供编译参数 |
| `.vscode/tasks.json` | 提供 VS Code 里的编译和烧录任务 |

## 如果红线还在，怎么处理

1. 在 VS Code 里按 `Ctrl+Shift+P`。
2. 执行：

```text
Developer: Reload Window
```

3. 如果装的是 Microsoft C/C++ 插件，再执行：

```text
C/C++: Reset IntelliSense Database
```

4. 确认当前打开的是工程根目录：

```text
C:\Users\qpzms\Desktop\STM32\TI\TI_3507_User
```

不要只打开 `User` 或 `User/Src` 子目录，否则 `.vscode` 配置不会按预期生效。

## 在 VS Code 里编译和烧录

按 `Ctrl+Shift+P`，选择：

```text
Tasks: Run Task
```

可用任务：

| 任务 | 作用 |
|---|---|
| `build gcc` | 编译生成 `firmware.elf` 和 `firmware.hex` |
| `flash cmsis-dap` | 先编译，再通过 CMSIS-DAP/OpenOCD 烧录 |

也可以直接在终端运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\build_user_gcc.ps1
powershell -ExecutionPolicy Bypass -File .\flash_cmsis_dap.ps1
```

## 怎么判断红线是不是真错误

以编译结果为准：

- 如果 `build_user_gcc.ps1` 显示 `BUILD_OK`，说明代码能编译。
- 如果 VS Code 红线还在，多半是 IntelliSense 配置或缓存问题。
- 如果脚本编译失败，那才需要按终端里的 gcc 报错去改代码。

