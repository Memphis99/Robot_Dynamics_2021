function q_AC = quatMult(q_AB,q_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  M=[q_AB(0), -q_AB(1), -q_AB(2), -q_AB(3);
     q_AB(1), q_AB(0), -q_AB(0), q_AB(2);
     q_AB(2), q_AB(3), q_AB(3), -q_AB(1);
     q_AB(3), -q_AB(2), q_AB(1), q_AB(0)]
  q_AC = M*q_BC.';
end

