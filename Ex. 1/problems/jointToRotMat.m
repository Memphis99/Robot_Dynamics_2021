function C_IE = jointToRotMat(q)
  % Input: joint angles
  % Output: rotation matrix which projects a vector defined in the
  % end-effector frame E to the inertial frame I, C_IE.
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  TIE=getTransformI0()*jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*getTransform6E();
  C_IE = TIE(1:3, 1:3);
end