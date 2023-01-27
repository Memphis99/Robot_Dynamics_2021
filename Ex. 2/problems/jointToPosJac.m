function J_P = jointToPosJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in I frame.

  % Compute the translational jacobian.
  
  TI0=getTransformI0_solution();
  T01=jointToTransform01_solution(q);
  T12=jointToTransform12_solution(q);
  T23=jointToTransform23_solution(q);
  T34=jointToTransform34_solution(q);
  T45=jointToTransform45_solution(q);
  T56=jointToTransform56_solution(q);
  T6E=getTransform6E_solution();
  
  TI1=TI0*T01; TI2=TI1*T12; TI3=TI2*T23; TI4=TI3*T34; TI5=TI4*T45; TI6=TI5*T56; TIE=TI6*T6E;
  
  T5E=T56*T6E; T4E=T45*T5E; T3E=T34*T4E; T2E=T23*T3E; T1E=T12*T2E;
  
  r6E=T6E(1:3, 4);  r5E=T5E(1:3, 4); r4E=T4E(1:3, 4); r3E=T3E(1:3, 4); r2E=T2E(1:3, 4); r1E=T1E(1:3, 4);
  
  n=jointToRotJac(q);
  
  
  J_P = [n(:,1).*r1E,  n(:,2).*r2E, n(:,3).*r3E, n(:,4).*r4E, n(:,5).*r5E, n(:,6).*r6E];
    
end