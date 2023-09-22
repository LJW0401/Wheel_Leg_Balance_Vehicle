%toolboxname = 'MATLAB Coder'; % 代码生成工具箱的名称
%if ~matlab.addons.isAddonEnabled(toolboxname)
%    matlab.addons.enableAddon(toolboxname);
%end
% 生成 C 代码
cfg = coder.config('dll'); % 配置生成选项，可以根据需要进行调整
cfg.TargetLang = 'C'; % 指定目标语言为 C
cfg.GenCodeOnly = true; % 生成代码而不是编译
codegen -config cfg leg_pos -o leg_pos -args {coder.typeof(single(0.0)),coder.typeof(single(0.0))} % leg_pos_ljw 是要生成的函数名称，-args 中的参数是函数输入的数据类型
codegen -config cfg leg_spd -o leg_spd -args {coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0))} % 同上，根据函数参数调整
codegen -config cfg leg_conv -o leg_conv -args {coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0))} % 同上，根据函数参数调整
