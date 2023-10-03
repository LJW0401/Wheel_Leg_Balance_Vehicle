%toolboxname = 'MATLAB Coder'; % 代码生成工具箱的名称
%if ~matlab.addons.isAddonEnabled(toolboxname)
%    matlab.addons.enableAddon(toolboxname);
%end

% 打开 MATLAB Coder
%coder
% 生成 C 代码
cfg = coder.config('dll'); % 配置生成选项，可以根据需要进行调整
cfg.TargetLang = 'C'; % 指定目标语言为 C
cfg.GenCodeOnly = true; % 生成代码而不是编译
%cfg.CustomSourceCode = '#define CUSTOM_ATAN2(u0, u1) atan2(u0, u1)';

disp("正在生成LegPos");
codegen -config cfg LegPos -args {coder.typeof(single(0.0)),coder.typeof(single(0.0))} % LegPos 是要生成的函数名称，-args 中的参数是函数输入的数据类型

disp("正在生成LegSpd");
codegen -config cfg LegSpd -args {coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0))} % 同上，根据函数参数调整

disp("正在生成LegConv");
codegen -config cfg LegConv -args {coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0)),coder.typeof(single(0.0))} % 同上，根据函数参数调整

disp("正在生成LQR_K");
codegen -config cfg LQR_K -args {coder.typeof(single(0.0))}