function T01 = jointToTransform01(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T01 = [cos(q(1)), -sin(q(1)), 0, 0;
         sin(q(1)), cos(q(1)), 0, 0;
         0, 0, 1, 0.145;
         0, 0, 0, 1];
end