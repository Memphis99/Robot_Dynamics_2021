function out1 = T_IF_fun(in1)
%T_IF_FUN
%    OUT1 = T_IF_FUN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    09-Nov-2021 16:57:12

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = q1+q2;
t3 = cos(t2);
t4 = q3+t2;
t5 = sin(t2);
t6 = cos(t4);
t7 = sin(t4);
out1 = reshape([0.0,t6,t7,0.0,0.0,-t7,t6,0.0,1.0,0.0,0.0,0.0,1.0./5.0,t3.*(7.0./2.5e+1)-t5./2.5e+1+t6.*(3.0./2.0e+1)+cos(q1).*(3.0./1.0e+1)+1.0./5.0,t3./2.5e+1+t5.*(7.0./2.5e+1)+t7.*(3.0./2.0e+1)+sin(q1).*(3.0./1.0e+1)+1.0./2.0,1.0],[4,4]);
