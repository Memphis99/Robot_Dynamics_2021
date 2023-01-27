function B_r = rotVecWithQuat(q_BA,A_r)
  % Input: the orientation quaternion and the coordinate of the vector to be mapped
  % Output: the coordinates of the vector in the target frame
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  P_A=[0;
       A_r(1);
       A_r(2);
       A_r(3)]
  P_B=q_BA*P_A*q_BA.';
  B_r = q_BA(2:4, 1).';
end
