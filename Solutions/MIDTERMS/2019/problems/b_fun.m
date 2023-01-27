function b = b_fun(in1,in2)
%B_FUN
%    B = B_FUN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    09-Nov-2021 16:57:14

dq1 = in2(1,:);
dq2 = in2(2,:);
dq3 = in2(3,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q2);
t3 = cos(q3);
t4 = sin(q2);
t5 = sin(q3);
t6 = q2+q3;
t7 = dq1.^2;
t8 = dq2.^2;
t9 = dq3.^2;
t10 = sin(t6);
t11 = dq1.*dq3.*t3.*(3.0./2.0e+2);
t12 = dq2.*dq3.*t3.*(3.0./2.0e+2);
t13 = dq1.*dq3.*t5.*(2.1e+1./2.0e+2);
t14 = dq2.*dq3.*t5.*(2.1e+1./2.0e+2);
t15 = t3.*t9.*(3.0./4.0e+2);
t16 = t5.*t9.*(2.1e+1./4.0e+2);
t17 = -t13;
t18 = -t14;
t19 = -t16;
t20 = t7.*t10.*(9.0./1.6e+2);
mt1 = [t11+t12+t15+t17+t18+t19-t2.*t8.*1.58625e-1-t4.*t8.*1.70625e-1-t8.*t10.*(9.0./1.6e+2)-t9.*t10.*(9.0./1.6e+2)-dq1.*dq2.*t2.*3.1725e-1-dq1.*dq2.*t4.*(2.73e+2./8.0e+2)-dq1.*dq2.*t10.*(9.0./8.0e+1)-dq1.*dq3.*t10.*(9.0./8.0e+1)-dq2.*dq3.*t10.*(9.0./8.0e+1),t11+t12+t15+t17+t18+t19+t20+t2.*t7.*1.58625e-1+t4.*t7.*1.70625e-1];
mt2 = [t20-t3.*t7.*(3.0./4.0e+2)-t3.*t8.*(3.0./4.0e+2)+t5.*t7.*(2.1e+1./4.0e+2)+t5.*t8.*(2.1e+1./4.0e+2)-dq1.*dq2.*t3.*(3.0./2.0e+2)+dq1.*dq2.*t5.*(2.1e+1./2.0e+2)];
b = reshape([mt1,mt2],3,1);
