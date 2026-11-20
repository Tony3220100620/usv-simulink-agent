%
%=============================================================================
% @brief   USV - Unmanned Surface Vehicle 无人船模型
% @param   ~: 时间, 未用到
% @param   x: 状态向量, 6×1, 包含：
%          纵荡位置 x_u
%          横荡位置 y
%          航向角 ψ
%          纵荡速度 u
%          横荡速度 v
%          航向角速度 r
% @param   u: 输入向量, 3×1, 包含：
%          纵荡力 X
%          横荡力 Y
%          航向力矩 N
% @param   flag: 执行阶段标志
% @param   cfg: 配置结构体, 包含：
%          x0: 状态向量初值, 6×1
%          MRB: 本体惯量矩阵, 3×3
%          MA: 附加惯量矩阵, 3×3
%          Dl: 线性阻尼矩阵, 3×3
%          Dn: 非线性阻尼矩阵, 3×3
% @retval  sys: 输出, 依 flag 而定: 0-初始化参数; 1-状态导数; 3-当前状态
% @retval  x0: 状态向量初值, 6×1
% @retval  str: 保留空字段
% @retval  ts: 采样时间, [period offset], 连续系统设为 [0 0]
% @note    参考文献: [1] FOSSEN T I. Handbook of Marine Craft Hydrodynamics 
%          and Motion Control[M]. Hoboken, NJ, USA: John Wiley & Sons, 2011.
%=============================================================================
%
function [sys,x0,str,ts] = USV(~,x,u,flag,cfg)

switch flag
    case 0 
        % 初始化                               
        [sys,x0,str,ts] = mdlInitializeSizes(cfg);
    case 1 
        % 计算状态导数
        sys = mdlDerivatives(x,u,cfg);
    case 3
        % 输出
        sys = mdlOutputs(x);
    case {2 4 9}
        % 未使用的 flag 值
        sys = [];
    otherwise
        % 错误处理
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

% end USV

%
%=============================================================================
% @brief   模块初始化函数
% @note    设置本模块的输入/输出端口数量、状态个数、采样时间等
%=============================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes(cfg)

sizes = simsizes;
sizes.NumContStates = 6;    % 状态向量共 6 个成员: x_u, y, ψ, u, v, r 
sizes.NumInputs = 3;        % 输入向量共 3 个成员: X, Y, N
sizes.NumOutputs = 6;       % 输出向量共 6 个成员: x_u, y, ψ, u, v, r 
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = cfg.x0;
str = [];
ts = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% @brief   状态导数计算函数
% @note    计算位姿和速度导数
%=============================================================================
%
function sys = mdlDerivatives(x,u,cfg)

% 计算转换矩阵
J = eulerang(0,0,x(3));
J = J([1,2,6],[1,2,6]);

% 获取 USV 速度
nu = x(4 : 6);

% 获取执行器力与力矩
tau = u(1 : 3);

% 计算水动力参数
MRB = cfg.MRB;
MA = cfg.MA;
M = MRB + MA;
CRB = m2c(MRB, nu);
CA = m2c(MA, nu);
D = cfg.Dl + cfg.Dn*diag(abs(nu));

% 计算位姿和速度导数
eta_dot = J*nu;
nu_dot = M\(tau - CRB*nu - CA*nu - D*nu);

% 输出位姿和速度导数
sys = [eta_dot; nu_dot];

% end mdlDerivatives

%
%=============================================================================
% @brief   模块输出函数
% @note    输出位姿和速度
%=============================================================================
%
function sys = mdlOutputs(x)

% 限制姿态角范围
x(3) = wrapToPi(x(3));

% 输出位姿和速度
sys = x;

% end mdlOutputs
