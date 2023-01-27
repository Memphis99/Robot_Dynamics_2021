function T34 = jointToTransform34(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 4 to frame 3. T_34
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T34 = [1, 0, 0, 0.134;
         0, cos(q(4)), -sin(q(4)), 0;
         0, sin(q(4)), cos(q(4)), 0.070;
         0, 0, 0, 1];
end

