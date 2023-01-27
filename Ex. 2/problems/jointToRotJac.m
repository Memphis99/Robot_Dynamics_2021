function J_R = jointToRotJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector orientation which maps joint
  % velocities to end-effector angular velocities in I frame.

  % Compute the rotational jacobian.
  
  TI0=getTransformI0_solution();
  CI0=TI0(1:3, 1:3);
  
  T01=jointToTransform01_solution(q);
  C01=T01(1:3, 1:3);
  
  T12=jointToTransform12_solution(q);
  C12=T12(1:3, 1:3);
  
  T23=jointToTransform23_solution(q);
  C23=T23(1:3, 1:3);
  
  T34=jointToTransform34_solution(q);
  C34=T34(1:3, 1:3);
  
  T45=jointToTransform45_solution(q);
  C45=T45(1:3, 1:3);
  
  T56=jointToTransform56_solution(q);
  C56=T56(1:3, 1:3);
  
  T6E=getTransform6E_solution();
  C6E=T6E(1:3, 1:3);
  
  n1=[0,0,1].';
  n2=[0,1,0].';
  n3=[0,1,0].';
  n4=[1,0,0].';
  n5=[0,1,0].';
  n6=[1,0,0].';
  
  
  J_R = [C01*n1, C01*C12*n2, C01*C12*C23*n3, C01*C12*C23*C34*n4, C01*C12*C23*C34*C45*n5, C01*C12*C23*C34*C45*C56*n6];
  
  
end
